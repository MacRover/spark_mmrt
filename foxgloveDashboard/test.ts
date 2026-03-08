/*
  Spark MAX -> Foxglove User Script

  Goal:
  Decode REV Spark MAX status frames on CAN and publish graph-friendly numeric fields.

  Output topic:
  - /sparkmax/ui

  Plot examples:
  - /sparkmax/ui.metrics.currentA
  - /sparkmax/ui.metrics.voltageV
  - /sparkmax/ui.devices.id_01.metrics.currentA
  - /sparkmax/ui.devices.id_02.metrics.primaryEncoderVelocity
*/

export const inputs = [
  "/can0/received_frames",
];

export const output = "/sparkmax/ui";

const STATUS0_BASE = 0x0205b800;
const STATUS1_BASE = 0x0205b840;
const STATUS2_BASE = 0x0205b880;
const STATUS3_BASE = 0x0205b8c0;
const STATUS4_BASE = 0x0205b900;
const STATUS5_BASE = 0x0205b940;
const STATUS6_BASE = 0x0205b980;
const STATUS7_BASE = 0x0205b9c0;
const STATUS8_BASE = 0x0205ba00;
const STATUS9_BASE = 0x0205ba40;

const DEVICE_ID_MASK = 0x3f;
const EXTENDED_ID_MASK = 0x1fffffff;

const APPLIED_OUTPUT_SCALE = 0.00003082369457075716;
const STATUS0_VOLTAGE_SCALE = 0.0073260073260073;
const STATUS0_CURRENT_SCALE = 0.0366300366300366;
const STATUS3_ANALOG_VOLTAGE_SCALE = 0.0048973607038123;
const STATUS3_ANALOG_VELOCITY_SCALE = 0.007812026887906498;
const STATUS6_UNADJUSTED_DUTY_SCALE = 0.00001541161211566339;

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
  appliedOutput: number;
  voltageV: number;
  currentA: number;
  motorTempC: number;
  hardForwardLimit: boolean;
  hardReverseLimit: boolean;
  softForwardLimit: boolean;
  softReverseLimit: boolean;
  inverted: boolean;
  primaryHeartbeatLock: boolean;
};

type Status1 = {
  otherFault: boolean;
  motorTypeFault: boolean;
  sensorFault: boolean;
  canFault: boolean;
  temperatureFault: boolean;
  drvFault: boolean;
  escEepromFault: boolean;
  firmwareFault: boolean;
  brownoutWarning: boolean;
  overcurrentWarning: boolean;
  escEepromWarning: boolean;
  extEepromWarning: boolean;
  sensorWarning: boolean;
  stallWarning: boolean;
  hasResetWarning: boolean;
  otherWarning: boolean;
  otherStickyFault: boolean;
  motorTypeStickyFault: boolean;
  sensorStickyFault: boolean;
  canStickyFault: boolean;
  temperatureStickyFault: boolean;
  drvStickyFault: boolean;
  escEepromStickyFault: boolean;
  firmwareStickyFault: boolean;
  brownoutStickyWarning: boolean;
  overcurrentStickyWarning: boolean;
  escEepromStickyWarning: boolean;
  extEepromStickyWarning: boolean;
  sensorStickyWarning: boolean;
  stallStickyWarning: boolean;
  hasResetStickyWarning: boolean;
  otherStickyWarning: boolean;
  isFollower: boolean;
};

type Status2 = {
  primaryEncoderVelocity: number;
  primaryEncoderPosition: number;
};

type Status3 = {
  analogVoltage: number;
  analogVelocity: number;
  analogPosition: number;
};

type Status4 = {
  altEncoderVelocity: number;
  altEncoderPosition: number;
};

type Status5 = {
  dutyCycleEncoderVelocity: number;
  dutyCycleEncoderPosition: number;
};

type Status6 = {
  unadjustedDutyCycle: number;
  dutyCyclePeriod: number;
  dutyCycleNoSignal: boolean;
};

type Status7 = {
  iAccumulation: number;
};

type Status8 = {
  closedLoopSetPoint: number;
  isAtSetpoint: boolean;
  selectedPIDSlot: number;
};

type Status9 = {
  maxMotionPositionSetPoint: number;
  maxMotionVelocitySetPoint: number;
};

type DeviceState = {
  status0?: Status0;
  status1?: Status1;
  status2?: Status2;
  status3?: Status3;
  status4?: Status4;
  status5?: Status5;
  status6?: Status6;
  status7?: Status7;
  status8?: Status8;
  status9?: Status9;
};

type Metrics = {
  appliedOutput: number;
  voltageV: number;
  currentA: number;
  motorTempC: number;
  hardForwardLimit: number;
  hardReverseLimit: number;
  softForwardLimit: number;
  softReverseLimit: number;
  inverted: number;
  primaryHeartbeatLock: number;
  faultOther: number;
  faultMotorType: number;
  faultSensor: number;
  faultCan: number;
  faultTemperature: number;
  faultDrv: number;
  faultEscEeprom: number;
  faultFirmware: number;
  warningBrownout: number;
  warningOvercurrent: number;
  warningEscEeprom: number;
  warningExtEeprom: number;
  warningSensor: number;
  warningStall: number;
  warningHasReset: number;
  warningOther: number;
  stickyFaultOther: number;
  stickyFaultMotorType: number;
  stickyFaultSensor: number;
  stickyFaultCan: number;
  stickyFaultTemperature: number;
  stickyFaultDrv: number;
  stickyFaultEscEeprom: number;
  stickyFaultFirmware: number;
  stickyWarningBrownout: number;
  stickyWarningOvercurrent: number;
  stickyWarningEscEeprom: number;
  stickyWarningExtEeprom: number;
  stickyWarningSensor: number;
  stickyWarningStall: number;
  stickyWarningHasReset: number;
  stickyWarningOther: number;
  isFollower: number;
  activeFaultCount: number;
  activeWarningCount: number;
  primaryEncoderVelocity: number;
  primaryEncoderPosition: number;
  analogVoltage: number;
  analogVelocity: number;
  analogPosition: number;
  altEncoderVelocity: number;
  altEncoderPosition: number;
  dutyCycleEncoderVelocity: number;
  dutyCycleEncoderPosition: number;
  unadjustedDutyCycle: number;
  dutyCyclePeriod: number;
  dutyCycleNoSignal: number;
  iAccumulation: number;
  closedLoopSetPoint: number;
  isAtSetpoint: number;
  selectedPIDSlot: number;
  maxMotionPositionSetPoint: number;
  maxMotionVelocitySetPoint: number;
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

function signExtend(value: number, bitLen: number): number {
  if (bitLen >= 32) {
    return value | 0;
  }
  const signMask = 1 << (bitLen - 1);
  const truncated = value & ((1 << bitLen) - 1);
  return (truncated ^ signMask) - signMask;
}

const f32Scratch = new ArrayBuffer(4);
const f32View = new DataView(f32Scratch);

function u32ToFloat(raw: number): number {
  f32View.setUint32(0, raw >>> 0, true);
  return f32View.getFloat32(0, true);
}

function bool01(v: boolean): number {
  return v ? 1 : 0;
}

function decodeStatus0(data: number[]): Status0 {
  const appliedOutputRaw = signExtend(getBits(data, 0, 16), 16);
  const voltageRaw = getBits(data, 16, 12);
  const currentRaw = getBits(data, 28, 12);

  return {
    appliedOutput: appliedOutputRaw * APPLIED_OUTPUT_SCALE,
    voltageV: voltageRaw * STATUS0_VOLTAGE_SCALE,
    currentA: currentRaw * STATUS0_CURRENT_SCALE,
    motorTempC: getBits(data, 40, 8),
    hardForwardLimit: getBits(data, 48, 1) === 1,
    hardReverseLimit: getBits(data, 49, 1) === 1,
    softForwardLimit: getBits(data, 50, 1) === 1,
    softReverseLimit: getBits(data, 51, 1) === 1,
    inverted: getBits(data, 52, 1) === 1,
    primaryHeartbeatLock: getBits(data, 53, 1) === 1,
  };
}

function decodeStatus1(data: number[]): Status1 {
  return {
    otherFault: getBits(data, 0, 1) === 1,
    motorTypeFault: getBits(data, 1, 1) === 1,
    sensorFault: getBits(data, 2, 1) === 1,
    canFault: getBits(data, 3, 1) === 1,
    temperatureFault: getBits(data, 4, 1) === 1,
    drvFault: getBits(data, 5, 1) === 1,
    escEepromFault: getBits(data, 6, 1) === 1,
    firmwareFault: getBits(data, 7, 1) === 1,
    brownoutWarning: getBits(data, 16, 1) === 1,
    overcurrentWarning: getBits(data, 17, 1) === 1,
    escEepromWarning: getBits(data, 18, 1) === 1,
    extEepromWarning: getBits(data, 19, 1) === 1,
    sensorWarning: getBits(data, 20, 1) === 1,
    stallWarning: getBits(data, 21, 1) === 1,
    hasResetWarning: getBits(data, 22, 1) === 1,
    otherWarning: getBits(data, 23, 1) === 1,
    otherStickyFault: getBits(data, 24, 1) === 1,
    motorTypeStickyFault: getBits(data, 25, 1) === 1,
    sensorStickyFault: getBits(data, 26, 1) === 1,
    canStickyFault: getBits(data, 27, 1) === 1,
    temperatureStickyFault: getBits(data, 28, 1) === 1,
    drvStickyFault: getBits(data, 29, 1) === 1,
    escEepromStickyFault: getBits(data, 30, 1) === 1,
    firmwareStickyFault: getBits(data, 31, 1) === 1,
    brownoutStickyWarning: getBits(data, 40, 1) === 1,
    overcurrentStickyWarning: getBits(data, 41, 1) === 1,
    escEepromStickyWarning: getBits(data, 42, 1) === 1,
    extEepromStickyWarning: getBits(data, 43, 1) === 1,
    sensorStickyWarning: getBits(data, 44, 1) === 1,
    stallStickyWarning: getBits(data, 45, 1) === 1,
    hasResetStickyWarning: getBits(data, 46, 1) === 1,
    otherStickyWarning: getBits(data, 47, 1) === 1,
    isFollower: getBits(data, 48, 1) === 1,
  };
}

function decodeStatus2(data: number[]): Status2 {
  return {
    primaryEncoderVelocity: u32ToFloat(getBits(data, 0, 32)),
    primaryEncoderPosition: u32ToFloat(getBits(data, 32, 32)),
  };
}

function decodeStatus3(data: number[]): Status3 {
  const analogVoltageRaw = getBits(data, 0, 10);
  const analogVelocityRaw = signExtend(getBits(data, 10, 22), 22);
  return {
    analogVoltage: analogVoltageRaw * STATUS3_ANALOG_VOLTAGE_SCALE,
    analogVelocity: analogVelocityRaw * STATUS3_ANALOG_VELOCITY_SCALE,
    analogPosition: u32ToFloat(getBits(data, 32, 32)),
  };
}

function decodeStatus4(data: number[]): Status4 {
  return {
    altEncoderVelocity: u32ToFloat(getBits(data, 0, 32)),
    altEncoderPosition: u32ToFloat(getBits(data, 32, 32)),
  };
}

function decodeStatus5(data: number[]): Status5 {
  return {
    dutyCycleEncoderVelocity: u32ToFloat(getBits(data, 0, 32)),
    dutyCycleEncoderPosition: u32ToFloat(getBits(data, 32, 32)),
  };
}

function decodeStatus6(data: number[]): Status6 {
  return {
    unadjustedDutyCycle: getBits(data, 0, 16) * STATUS6_UNADJUSTED_DUTY_SCALE,
    dutyCyclePeriod: getBits(data, 16, 16),
    dutyCycleNoSignal: getBits(data, 32, 1) === 1,
  };
}

function decodeStatus7(data: number[]): Status7 {
  return {
    iAccumulation: u32ToFloat(getBits(data, 0, 32)),
  };
}

function decodeStatus8(data: number[]): Status8 {
  return {
    closedLoopSetPoint: u32ToFloat(getBits(data, 0, 32)),
    isAtSetpoint: getBits(data, 32, 1) === 1,
    selectedPIDSlot: getBits(data, 33, 4),
  };
}

function decodeStatus9(data: number[]): Status9 {
  return {
    maxMotionPositionSetPoint: u32ToFloat(getBits(data, 0, 32)),
    maxMotionVelocitySetPoint: u32ToFloat(getBits(data, 32, 32)),
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
    case STATUS1_BASE:
      state.status1 = decodeStatus1(data);
      return "status1";
    case STATUS2_BASE:
      state.status2 = decodeStatus2(data);
      return "status2";
    case STATUS3_BASE:
      state.status3 = decodeStatus3(data);
      return "status3";
    case STATUS4_BASE:
      state.status4 = decodeStatus4(data);
      return "status4";
    case STATUS5_BASE:
      state.status5 = decodeStatus5(data);
      return "status5";
    case STATUS6_BASE:
      state.status6 = decodeStatus6(data);
      return "status6";
    case STATUS7_BASE:
      state.status7 = decodeStatus7(data);
      return "status7";
    case STATUS8_BASE:
      state.status8 = decodeStatus8(data);
      return "status8";
    case STATUS9_BASE:
      state.status9 = decodeStatus9(data);
      return "status9";
    default:
      return "";
  }
}
function makeEmptyMetrics(): Metrics {
  return {
    appliedOutput: Number.NaN,
    voltageV: Number.NaN,
    currentA: Number.NaN,
    motorTempC: Number.NaN,
    hardForwardLimit: 0,
    hardReverseLimit: 0,
    softForwardLimit: 0,
    softReverseLimit: 0,
    inverted: 0,
    primaryHeartbeatLock: 0,
    faultOther: 0,
    faultMotorType: 0,
    faultSensor: 0,
    faultCan: 0,
    faultTemperature: 0,
    faultDrv: 0,
    faultEscEeprom: 0,
    faultFirmware: 0,
    warningBrownout: 0,
    warningOvercurrent: 0,
    warningEscEeprom: 0,
    warningExtEeprom: 0,
    warningSensor: 0,
    warningStall: 0,
    warningHasReset: 0,
    warningOther: 0,
    stickyFaultOther: 0,
    stickyFaultMotorType: 0,
    stickyFaultSensor: 0,
    stickyFaultCan: 0,
    stickyFaultTemperature: 0,
    stickyFaultDrv: 0,
    stickyFaultEscEeprom: 0,
    stickyFaultFirmware: 0,
    stickyWarningBrownout: 0,
    stickyWarningOvercurrent: 0,
    stickyWarningEscEeprom: 0,
    stickyWarningExtEeprom: 0,
    stickyWarningSensor: 0,
    stickyWarningStall: 0,
    stickyWarningHasReset: 0,
    stickyWarningOther: 0,
    isFollower: 0,
    activeFaultCount: 0,
    activeWarningCount: 0,
    primaryEncoderVelocity: Number.NaN,
    primaryEncoderPosition: Number.NaN,
    analogVoltage: Number.NaN,
    analogVelocity: Number.NaN,
    analogPosition: Number.NaN,
    altEncoderVelocity: Number.NaN,
    altEncoderPosition: Number.NaN,
    dutyCycleEncoderVelocity: Number.NaN,
    dutyCycleEncoderPosition: Number.NaN,
    unadjustedDutyCycle: Number.NaN,
    dutyCyclePeriod: Number.NaN,
    dutyCycleNoSignal: 0,
    iAccumulation: Number.NaN,
    closedLoopSetPoint: Number.NaN,
    isAtSetpoint: 0,
    selectedPIDSlot: 0,
    maxMotionPositionSetPoint: Number.NaN,
    maxMotionVelocitySetPoint: Number.NaN,
  };
}

function buildMetrics(state: DeviceState): Metrics {
  const metrics = makeEmptyMetrics();


  if (state.status0) {
    metrics.appliedOutput = state.status0.appliedOutput;
    metrics.voltageV = state.status0.voltageV;
    metrics.currentA = state.status0.currentA;
    metrics.motorTempC = state.status0.motorTempC;
    metrics.hardForwardLimit = bool01(state.status0.hardForwardLimit);
    metrics.hardReverseLimit = bool01(state.status0.hardReverseLimit);
    metrics.softForwardLimit = bool01(state.status0.softForwardLimit);
    metrics.softReverseLimit = bool01(state.status0.softReverseLimit);
    metrics.inverted = bool01(state.status0.inverted);
    metrics.primaryHeartbeatLock = bool01(state.status0.primaryHeartbeatLock);
  }

  if (state.status1) {
    metrics.faultOther = bool01(state.status1.otherFault);
    metrics.faultMotorType = bool01(state.status1.motorTypeFault);
    metrics.faultSensor = bool01(state.status1.sensorFault);
    metrics.faultCan = bool01(state.status1.canFault);
    metrics.faultTemperature = bool01(state.status1.temperatureFault);
    metrics.faultDrv = bool01(state.status1.drvFault);
    metrics.faultEscEeprom = bool01(state.status1.escEepromFault);
    metrics.faultFirmware = bool01(state.status1.firmwareFault);

    metrics.warningBrownout = bool01(state.status1.brownoutWarning);
    metrics.warningOvercurrent = bool01(state.status1.overcurrentWarning);
    metrics.warningEscEeprom = bool01(state.status1.escEepromWarning);
    metrics.warningExtEeprom = bool01(state.status1.extEepromWarning);
    metrics.warningSensor = bool01(state.status1.sensorWarning);
    metrics.warningStall = bool01(state.status1.stallWarning);
    metrics.warningHasReset = bool01(state.status1.hasResetWarning);
    metrics.warningOther = bool01(state.status1.otherWarning);

    metrics.stickyFaultOther = bool01(state.status1.otherStickyFault);
    metrics.stickyFaultMotorType = bool01(state.status1.motorTypeStickyFault);
    metrics.stickyFaultSensor = bool01(state.status1.sensorStickyFault);
    metrics.stickyFaultCan = bool01(state.status1.canStickyFault);
    metrics.stickyFaultTemperature = bool01(state.status1.temperatureStickyFault);
    metrics.stickyFaultDrv = bool01(state.status1.drvStickyFault);
    metrics.stickyFaultEscEeprom = bool01(state.status1.escEepromStickyFault);
    metrics.stickyFaultFirmware = bool01(state.status1.firmwareStickyFault);

    metrics.stickyWarningBrownout = bool01(state.status1.brownoutStickyWarning);
    metrics.stickyWarningOvercurrent = bool01(state.status1.overcurrentStickyWarning);
    metrics.stickyWarningEscEeprom = bool01(state.status1.escEepromStickyWarning);
    metrics.stickyWarningExtEeprom = bool01(state.status1.extEepromStickyWarning);
    metrics.stickyWarningSensor = bool01(state.status1.sensorStickyWarning);
    metrics.stickyWarningStall = bool01(state.status1.stallStickyWarning);
    metrics.stickyWarningHasReset = bool01(state.status1.hasResetStickyWarning);
    metrics.stickyWarningOther = bool01(state.status1.otherStickyWarning);
    metrics.isFollower = bool01(state.status1.isFollower);

    metrics.activeFaultCount =
      metrics.faultOther +
      metrics.faultMotorType +
      metrics.faultSensor +
      metrics.faultCan +
      metrics.faultTemperature +
      metrics.faultDrv +
      metrics.faultEscEeprom +
      metrics.faultFirmware;

    metrics.activeWarningCount =
      metrics.warningBrownout +
      metrics.warningOvercurrent +
      metrics.warningEscEeprom +
      metrics.warningExtEeprom +
      metrics.warningSensor +
      metrics.warningStall +
      metrics.warningHasReset +
      metrics.warningOther;
  }

  if (state.status2) {
    metrics.primaryEncoderVelocity = state.status2.primaryEncoderVelocity;
    metrics.primaryEncoderPosition = state.status2.primaryEncoderPosition;
  }

  if (state.status3) {
    metrics.analogVoltage = state.status3.analogVoltage;
    metrics.analogVelocity = state.status3.analogVelocity;
    metrics.analogPosition = state.status3.analogPosition;
  }

  if (state.status4) {
    metrics.altEncoderVelocity = state.status4.altEncoderVelocity;
    metrics.altEncoderPosition = state.status4.altEncoderPosition;
  }

  if (state.status5) {
    metrics.dutyCycleEncoderVelocity = state.status5.dutyCycleEncoderVelocity;
    metrics.dutyCycleEncoderPosition = state.status5.dutyCycleEncoderPosition;
  }

  if (state.status6) {
    metrics.unadjustedDutyCycle = state.status6.unadjustedDutyCycle;
    metrics.dutyCyclePeriod = state.status6.dutyCyclePeriod;
    metrics.dutyCycleNoSignal = bool01(state.status6.dutyCycleNoSignal);
  }

  if (state.status7) {
    metrics.iAccumulation = state.status7.iAccumulation;
  }

  if (state.status8) {
    metrics.closedLoopSetPoint = state.status8.closedLoopSetPoint;
    metrics.isAtSetpoint = bool01(state.status8.isAtSetpoint);
    metrics.selectedPIDSlot = state.status8.selectedPIDSlot;
  }

  if (state.status9) {
    metrics.maxMotionPositionSetPoint = state.status9.maxMotionPositionSetPoint;
    metrics.maxMotionVelocitySetPoint = state.status9.maxMotionVelocitySetPoint;
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
