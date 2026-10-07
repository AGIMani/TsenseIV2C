// Shared serial line configuration. CDC often ignores baud; UART must match firmware.
export function serialOptions(baudRate = 115200) {
  if (![115200,230400,460800,921600,1000000,2000000,3000000].includes(baudRate)) throw Error('不支持的串口速率');
  return {baudRate,dataBits:8,stopBits:1,parity:'none',flowControl:'none',bufferSize:65536};
}
export function portDescription(port) {
  const {usbVendorId:vid,usbProductId:pid} = port.getInfo?.() || {};
  if (vid===0x0483&&pid===0x5740) return 'H7 原生 USB CDC';
  if (vid===0x0483&&pid===0x374f) return 'ST-Link 虚拟串口（可能经过 UART）';
  return vid===undefined?'串口类型未知':`USB ${vid.toString(16)}:${pid?.toString(16)}`;
}
