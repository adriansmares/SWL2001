function decode4Bytes(bytes, offset) {
  return bytes[offset] | (bytes[offset + 1] << 8) | (bytes[offset + 2] << 16) | (bytes[offset + 3] << 24);
}

function decodeUplink(input) {
  const data = {};
  switch (input.fPort) {
    case 102:
      const parameters = {};
      data.parameters = parameters;
      const mcu = {};
      data.mcu = mcu;
      switch (input.bytes[0]) {
        case 0x01:
          data.parameters = {
            gnssScanMode: input.bytes[1] === 0 ? "static" : "mobile",
            gnssScanRate: decode4Bytes(input.bytes, 2),
            wifiScanRate: decode4Bytes(input.bytes, 6),
          };
          data.mcu = {
            temperature: input.bytes[10],
          };
          data.deviceManagement = {
            reportingRate: {
              unit: function (b) {
                switch (b) {
                  case 0x00:
                    return "second";
                  case 0x01:
                    return "day";
                  case 0x02:
                    return "hour";
                  default:
                    return "minute";
                }
              }(input.bytes[11]),
              value: input.bytes[12]
            },
          }
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  return {
    data: data,
    warnings: [],
    errors: []
  };
}