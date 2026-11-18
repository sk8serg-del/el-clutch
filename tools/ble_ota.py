"""BLE OTA для Stark Clutch с компьютера (тот же протокол, что у страницы настроек).

  python tools/ble_ota.py --scan                 — найти платы, показать версию и метку
  python tools/ble_ota.py --all                  — обновить все найденные до последней сборки в firmware/
  python tools/ble_ota.py --addr AA:BB:.. [--file X.bin]

Файл выбирается по метке платы (GEN1/GEN2) и чипу из версии. Плата сама ещё раз
сверяет чип и метку. Нужен bleak (pip install bleak).
"""
import argparse, asyncio, glob, os, re, sys, time, zlib
from bleak import BleakClient, BleakScanner

SERVICE = '19b10000-e8f2-537e-4f6c-d104768a1214'
FW_VER  = '19b10020-e8f2-537e-4f6c-d104768a1214'
CTRL    = '19b10021-e8f2-537e-4f6c-d104768a1214'
DATA    = '19b10022-e8f2-537e-4f6c-d104768a1214'
HW      = '19b10051-e8f2-537e-4f6c-d104768a1214'
SENSOR_CANDIDATES = None
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEGACY = {'CLUTCH': 'GEN1', 'CLUTCH_CAN': 'GEN2'}


def vnum(v):
    m = re.search(r'(\d+)\.(\d+)\.(\d+)', v or '')
    return tuple(map(int, m.groups())) if m else (0, 0, 0)


def latest_file(gen, chip):
    best = None
    for f in glob.glob(os.path.join(ROOT, 'firmware', f'{gen}_{chip}', f'{gen}_v*_{chip}.bin')):
        v = vnum(os.path.basename(f))
        if not best or v > best[0]:
            best = (v, f)
    return best[1] if best else None


def file_label(data):
    m = re.search(rb'STARKFWID\|(\w+)\|([CS]3)\|([\d.]+)\|', data)
    return (m.group(1).decode(), m.group(2).decode(), m.group(3).decode()) if m else (None, None, None)


async def scan(timeout=6):
    found = await BleakScanner.discover(timeout=timeout)
    return [d for d in found if (d.name or '') == 'Stark_Clutch']


async def read_info(client):
    ver = (await client.read_gatt_char(FW_VER)).decode(errors='replace')
    try:
        hw = (await client.read_gatt_char(HW)).decode(errors='replace')
    except Exception:
        hw = None
    return ver, hw


async def ota(client, data, ver):
    fast = vnum(ver) >= (2, 16, 11)
    msgs = asyncio.Queue()
    err = []

    def on_ctrl(_, b):
        m = bytes(b).decode(errors='replace')
        if m.startswith('ERROR'):
            err.append(m)
        msgs.put_nowait(m)

    async def wait_for(key, t=8):
        end = time.time() + t
        while True:
            left = end - time.time()
            if left <= 0:
                raise TimeoutError('timeout: ' + key)
            m = await asyncio.wait_for(msgs.get(), left)
            if m.startswith('ERROR'):
                raise RuntimeError(m)
            if m == key or m.startswith(key + ':'):
                return m

    await client.start_notify(CTRL, on_ctrl)
    payload = zlib.compress(data, 9) if fast else data
    cmd = (f'SIZEZ:{len(data)}' if fast else f'SIZE:{len(data)}').encode()
    await client.write_gatt_char(CTRL, cmd, response=True)
    ready = await wait_for('READY')
    if fast:
        mtu = int(ready.split(':')[1]) if ':' in ready else 23
        chunk = max(20, min(512, mtu - 3))
    else:
        chunk = 496
    total = len(payload)
    print(f'  sending {total // 1024} KB' + (f' (zlib from {len(data) // 1024} KB), chunk {chunk}' if fast else ''))
    t0 = time.time(); last = -1
    for i, off in enumerate(range(0, total, chunk)):
        if err:
            raise RuntimeError(err[0])
        part = payload[off:off + chunk]
        resp = (not fast) or (i % 24 == 23)
        await client.write_gatt_char(DATA, part, response=resp)
        pct = (off + len(part)) * 100 // total
        if pct != last and pct % 5 == 0:
            last = pct
            print(f'\r  {pct:3d}%  {time.time() - t0:5.1f} s', end='', flush=True)
    print()
    await asyncio.sleep(0.3)
    if err:
        raise RuntimeError(err[0])
    await client.write_gatt_char(CTRL, (f'APPLY:{total}' if fast else 'APPLY').encode(), response=True)
    await wait_for('DONE', 15)
    print(f'  DONE in {time.time() - t0:.0f} s, board is rebooting')


async def handle(dev, file_arg, dry):
    print(f'{dev.address}  ', end='')
    async with BleakClient(dev, timeout=20) as c:
        ver, hw = await read_info(c)
        chip = (re.search(r'-([CS]3)', ver) or [None, None])[1]
        gen = LEGACY.get(hw, hw)
        print(f'fw {ver}  label {hw}')
        f = file_arg or (latest_file(gen, chip) if gen and chip else None)
        if not f:
            print('  no firmware file for this board'); return
        data = open(f, 'rb').read()
        flab, fchip, fver = file_label(data)
        if fchip != chip or LEGACY.get(flab, flab) != gen:
            print(f'  {os.path.basename(f)} is {flab}/{fchip}, board is {hw}/{chip} — skipped'); return
        if flab != hw and vnum(ver) < (2, 16, 14):
            print(f'  board {ver} accepts only {hw} files over BLE — needs USB or a transition build, skipped'); return
        if not file_arg and vnum(fver) <= vnum(ver):
            print(f'  already up to date ({fver})'); return
        print(f'  -> {os.path.basename(f)}')
        if dry:
            return
        await ota(c, data, ver)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scan', action='store_true')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--addr')
    ap.add_argument('--file')
    a = ap.parse_args()
    devs = await scan()
    if a.addr:
        devs = [d for d in devs if d.address.upper() == a.addr.upper()]
    if not devs:
        sys.exit('no Stark_Clutch found')
    for d in devs:
        try:
            await handle(d, a.file, a.scan or not (a.all or a.addr))
        except Exception as e:
            print(f'  FAILED: {type(e).__name__}: {e!r}')


asyncio.run(main())
