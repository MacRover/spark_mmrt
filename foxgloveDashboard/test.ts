/*
  Spark MAX -> Foxglove User Script (slim mode)

  Currently enabled metrics only:
  - voltageV
  - currentA
  - dutyCycleEncoderPosition

  Plot examples:
  - /sparkmax/ui.metrics.voltageV
  - /sparkmax/ui.metrics.currentA
  - /sparkmax/ui.metrics.dutyCycleEncoderPosition
*/

export const inputs = ["/can0/received_frames"];
export const output = "/sparkmax/ui";

const STATUS0_BASE = 0x0205b800;
// const STATUS1_BASE = 0x0205b840; // disabled for slim payload
// const STATUS2_BASE = 0x0205b880; // disabled for slim payload
// const STATUS3_BASE = 0x0205b8c0; // disabled for slim payload
// const STATUS4_BASE = 0x0205b900; // disabled for slim payload
const STATUS5_BASE = 0x0205b940;
// const STATUS6_BASE = 0x0205b980; // disabled for slim payload
// const STATUS7_BASE = 0x0205b9c0; // disabled for slim payload
// const STATUS8_BASE = 0x0205ba00; // disabled for slim payload
// const STATUS9_BASE = 0x0205ba40; // disabled for slim payload

const DEVICE_ID_MASK = 0x3f;
const EXTENDED_ID_MASK = 0x1fffffff;

const STATUS0_VOLTAGE_SCALE = 0.0073260073260073;
const STATUS0_CURRENT_SCALE = 0.0366300366300366;

type TimeLike = { sec: number; nsec: number };

type ScriptEvent = {
  topic: string;
  message: unknown;
  receiveTime?: TimeLike;
};

type CanFrameLike = {
  id?: unknown;
  arbId?: unknown;
  can_id?: unknown;
  canId?: unknown;
  arbitration_id?: unknown;
  data?: unknown;
  bytes?: unknown;
  frame?: unknown;
};

type Status0 = {
  voltageV: number;
  currentA: number;
};

type Status5 = {
  dutyCycleEncoderPosition: number;
};

type DeviceState = {
  status0?: Status0;
  status5?: Status5;
};

type Metrics = {
  voltageV: number;
  currentA: number;
  dutyCycleEncoderPosition: number;
};

type SparkMaxUiDevice = {
  deviceId: number;
  metrics: Metrics;
};

type SparkMaxUiMessage = {
  valid: number;
  updatedDevice: string;
  updatedDeviceId: number;
  changedStatus: string;
  sourceTopic: string;
  arbId: number;
  receiveTimeSec: number;
  receiveTimeNsec: number;
  metrics: Metrics;
  devices: { [key: string]: SparkMaxUiDevice };
};

const stateByDevice = new Map<number, DeviceState>();

function asNumber(value: unknown): number | undefined {
  if (typeof value === "number" && Number.isFinite(value)) {
    return value;
  }
  if (typeof value === "bigint") {
    return Number(value);
  }
  if (typeof value === "string") {
    const t = value.trim();
    const isHex = /^0x[0-9a-f]+$/i.test(t);
    const parsed = isHex ? Number.parseInt(t, 16) : Number(t);
    if (Number.isFinite(parsed)) {
      return parsed;
    }
  }
  return undefined;
}

function normalizeByte(value: unknown): number {
  const n = asNumber(value);
  if (n === undefined) {
    return 0;
  }
  if (n <= 0) {
    return 0;
  }
  if (n >= 255) {
    return 255;
  }
  return n & 0xff;
}

function parseByteString(value: string): number[] | undefined {
  const trimmed = value.trim();
  if (!trimmed) {
    return undefined;
  }
  const compact = trimmed.replace(/[\s,_-]/g, "");
  if (!/^[0-9a-f]+$/i.test(compact) || compact.length % 2 !== 0) {
    return undefined;
  }

  const out: number[] = [];
  for (let i = 0; i < compact.length; i += 2) {
    out.push(Number.parseInt(compact.slice(i, i + 2), 16));
  }
  return out;
}

function asByteArray(value: unknown): number[] | undefined {
  if (Array.isArray(value)) {
    return value.map(normalizeByte);
  }
  if (typeof value === "string") {
    return parseByteString(value);
  }
  if (ArrayBuffer.isView(value)) {
    const view = value as ArrayBufferView;
    const bytes = new Uint8Array(view.buffer, view.byteOffset, view.byteLength);
    return Array.from(bytes, normalizeByte);
  }
  return undefined;
}

function normalizeCanFrame(
  message: unknown,
): { arbId: number; data: number[] } | undefined {
  if (!message || typeof message !== "object") {
    return;
  }

  const msg = message as CanFrameLike;
  if (msg.frame && typeof msg.frame === "object") {
    return normalizeCanFrame(msg.frame);
  }

  const rawId =
    asNumber(msg.arbId) ??
    asNumber(msg.id) ??
    asNumber(msg.can_id) ??
    asNumber(msg.canId) ??
    asNumber(msg.arbitration_id);

  if (rawId === undefined) {
    return;
  }

  const arbId = (rawId >>> 0) & EXTENDED_ID_MASK;
  const rawBytes = asByteArray(msg.data) ?? asByteArray(msg.bytes);
  if (!rawBytes) {
    return;
  }

  const data = new Array<number>(8).fill(0);
  const len = Math.min(rawBytes.length, 8);
  for (let i = 0; i < len; i += 1) {
    data[i] = rawBytes[i];
  }
  return { arbId, data };
}

function getBits(data: number[], bitPos: number, bitLen: number): number {
  let out = 0;
  for (let i = 0; i < bitLen; i += 1) {
    const b = bitPos + i;
    const byteIndex = Math.floor(b / 8);
    const bitIndex = b % 8;
    const bit = (data[byteIndex] >> bitIndex) & 1;
    out |= bit << i;
  }
  return out >>> 0;
}

const f32Scratch = new ArrayBuffer(4);
const f32View = new DataView(f32Scratch);

function u32ToFloat(raw: number): number {
  f32View.setUint32(0, raw >>> 0, true);
  return f32View.getFloat32(0, true);
}

function decodeStatus0(data: number[]): Status0 {
  const voltageRaw = getBits(data, 16, 12);
  const currentRaw = getBits(data, 28, 12);
  return {
    voltageV: voltageRaw * STATUS0_VOLTAGE_SCALE,
    currentA: currentRaw * STATUS0_CURRENT_SCALE,
  };
}

function decodeStatus5(data: number[]): Status5 {
  return {
    dutyCycleEncoderPosition: u32ToFloat(getBits(data, 32, 32)),
  };
}

function applyStatusUpdate(
  base: number,
  data: number[],
  state: DeviceState,
): string {
  switch (base) {
    case STATUS0_BASE:
      state.status0 = decodeStatus0(data);
      return "status0";
    case STATUS5_BASE:
      state.status5 = decodeStatus5(data);
      return "status5";
    default:
      // Status1/2/3/4/6/7/8/9 intentionally ignored in slim mode.
      return "";
  }
}

function makeEmptyMetrics(): Metrics {
  return {
    voltageV: Number.NaN,
    currentA: Number.NaN,
    dutyCycleEncoderPosition: Number.NaN,
  };
}

function buildMetrics(state: DeviceState): Metrics {
  const metrics = makeEmptyMetrics();

  if (state.status0) {
    metrics.voltageV = state.status0.voltageV;
    metrics.currentA = state.status0.currentA;
  }

  if (state.status5) {
    metrics.dutyCycleEncoderPosition = state.status5.dutyCycleEncoderPosition;
  }

  return metrics;
}

function deviceKey(deviceId: number): string {
  return `id_${deviceId.toString().padStart(2, "0")}`;
}

function makeBaseMessage(event: ScriptEvent, arbId: number): SparkMaxUiMessage {
  const sec = event.receiveTime ? event.receiveTime.sec : 0;
  const nsec = event.receiveTime ? event.receiveTime.nsec : 0;
  return {
    valid: 0,
    updatedDevice: "",
    updatedDeviceId: 0,
    changedStatus: "",
    sourceTopic: event.topic,
    arbId,
    receiveTimeSec: sec,
    receiveTimeNsec: nsec,
    metrics: makeEmptyMetrics(),
    devices: {},
  };
}

export default function script(event: ScriptEvent): SparkMaxUiMessage {
  const out = makeBaseMessage(event, 0);
  const frame = normalizeCanFrame(event.message);
  if (!frame) {
    out.changedStatus = "ignored.invalidFrame";
    return out;
  }
  out.arbId = frame.arbId;

  const deviceId = frame.arbId & DEVICE_ID_MASK;
  const base = frame.arbId & ~DEVICE_ID_MASK;

  let state = stateByDevice.get(deviceId);
  if (!state) {
    state = {};
    stateByDevice.set(deviceId, state);
  }

  const changedStatus = applyStatusUpdate(base, frame.data, state);
  if (!changedStatus) {
    out.changedStatus = "ignored.nonStatusFrame";
    return out;
  }

  const devices: { [key: string]: SparkMaxUiDevice } = {};
  for (const [id, s] of stateByDevice.entries()) {
    devices[deviceKey(id)] = {
      deviceId: id,
      metrics: buildMetrics(s),
    };
  }

  out.valid = 1;
  out.updatedDevice = deviceKey(deviceId);
  out.updatedDeviceId = deviceId;
  out.changedStatus = changedStatus;
  out.metrics = buildMetrics(state);
  out.devices = devices;
  return out;
}
