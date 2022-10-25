function encode4Bytes(x) {
  return [x & 0xFF, (x >> 8) & 0xFF, (x >> 16) & 0xFF, (x >> 24) & 0xFF];
}

function encodeDownlink(input) {
  const { parameters, commands, deviceManagement } = input.data;
  const bytes = [];

  if (parameters !== undefined) {
    const { gnssScanMode, gnssScanRate, wifiScanRate } = parameters;
    if (gnssScanMode !== undefined) {
      bytes.push(0x10, gnssScanMode == "static" ? 0x00 : 0x01);
    }
    if (gnssScanRate !== undefined) {
      bytes.push(0x11, ...encode4Bytes(gnssScanRate));
    }
    if (wifiScanRate !== undefined) {
      bytes.push(0x12, ...encode4Bytes(wifiScanRate));
    }
  }
  if (commands !== undefined) {
    const { requestParameters, requestGNSSScan, requestWiFiScan } = commands;
    if (requestParameters !== undefined) {
      bytes.push(0x01);
    }
    if (requestGNSSScan !== undefined) {
      const { delay } = requestGNSSScan;
      if (delay !== undefined) {
        bytes.push(0x20, ...encode4Bytes(delay));
      }
    }
    if (requestWiFiScan !== undefined) {
      const { delay } = requestWiFiScan;
      if (delay !== undefined) {
        bytes.push(0x21, ...encode4Bytes(delay));
      }
    }
  }
  if (deviceManagement !== undefined) {
    const { reportingRate } = deviceManagement;
    if (reportingRate !== undefined) {
      const { unit, rate } = reportingRate;
      if (unit !== undefined && rate !== undefined) {
        const encodedUnit = function (u) {
          switch (u) {
            case "second":
              return 0x00;
            case "day":
              return 0x01;
            case "hour":
              return 0x02;
            default:
              return 0x03;
          }
        }(unit);
        bytes.push(0x13, encodedUnit, rate & 0xFF);
      }
    }
  }

  return {
    bytes: bytes,
    fPort: 102,
    warnings: [],
    errors: []
  };
}

function decodeDownlink(input) {
  const data = {};
  switch (input.fPort) {
    case 150:
      break;
    case 199:
      break;
    default:
      data.bytes = input.bytes;
      break;
  }
  return {
    data: data,
    warnings: [],
    errors: []
  }
}