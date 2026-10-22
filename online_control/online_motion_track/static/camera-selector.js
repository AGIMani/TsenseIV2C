// Refresh browser-visible cameras on opening the selector, without starting tracking.
export function bindCameraSelector(select, {mediaDevices = navigator.mediaDevices, onError = () => {}} = {}) {
  let pending = null;
  let permissionRequested = false;

  const enumerate = async () => (await mediaDevices.enumerateDevices())
    .filter(device => device.kind === 'videoinput');

  async function scan() {
    if (!mediaDevices?.enumerateDevices) {
      if (permissionRequested) throw new Error('无法读取摄像头，请使用 HTTPS 或 localhost 打开页面。');
      return;
    }
    let devices = await enumerate();
    // Before camera permission, browsers may expose only an anonymous default
    // device. Ask only on user interaction, never on startup or devicechange.
    if (permissionRequested && (!devices.length || devices.some(device => !device.label || !device.deviceId))) {
      let permissionStream;
      try {
        permissionStream = await mediaDevices.getUserMedia({audio: false, video: true});
        devices = await enumerate();
      } catch (error) {
        if (error.name !== 'NotFoundError') throw error;
        devices = [];
        onError(error);
      } finally {
        permissionStream?.getTracks().forEach(track => track.stop());
      }
    }

    const entries = [{label: '默认摄像头', deviceId: ''}, ...devices.filter(device => device.deviceId)
      .map((device, index) => ({label: device.label || `摄像头 ${index + 1}`, deviceId: device.deviceId}))];
    // Keep the native popup and its current selection intact if nothing changed.
    const previous = [...select.options];
    if (previous.length === entries.length && entries.every((device, index) =>
      previous[index].value === device.deviceId && previous[index].textContent === device.label)) return;
    // Read after awaiting: the user may have changed selection during the scan.
    const selected = select.value;
    select.replaceChildren(...entries.map(device => {
      const option = select.ownerDocument.createElement('option');
      option.value = device.deviceId;
      option.textContent = device.label;
      return option;
    }));
    select.value = entries.some(device => device.deviceId === selected) ? selected : '';
  }

  function refresh({requestPermission = false} = {}) {
    permissionRequested ||= requestPermission;
    if (pending) return pending;
    select.setAttribute('aria-busy', 'true');
    pending = scan().finally(() => {
      pending = null;
      permissionRequested = false;
      select.removeAttribute('aria-busy');
    });
    return pending;
  }

  const open = () => {
    if (!select.disabled) refresh({requestPermission: true}).catch(onError);
  };
  select.addEventListener('pointerdown', event => {
    if (event.button === 0) open();
  });
  select.addEventListener('keydown', event => {
    if (!event.repeat && ['Enter', ' ', 'ArrowDown', 'ArrowUp', 'F4'].includes(event.key)) open();
  });
  mediaDevices?.addEventListener('devicechange', () => refresh().catch(onError));
  return {refresh};
}
