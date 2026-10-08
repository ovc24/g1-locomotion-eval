import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

NOMBRES = ['L_hip_pitch','L_hip_roll','L_hip_yaw','L_knee','L_ankle_pitch','L_ankle_roll',
           'R_hip_pitch','R_hip_roll','R_hip_yaw','R_knee','R_ankle_pitch','R_ankle_roll',
           'waist_yaw','waist_roll','waist_pitch',
           'L_sh_pitch','L_sh_roll','L_sh_yaw','L_elbow','L_wr_roll','L_wr_pitch','L_wr_yaw',
           'R_sh_pitch','R_sh_roll','R_sh_yaw','R_elbow','R_wr_roll','R_wr_pitch','R_wr_yaw']

bag = sys.argv[1].rstrip('/')
t_ini = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
t_fin = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
umbral = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0  # grados

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
tipos = {t.name: t.type for t in reader.get_all_topics_and_types()}
st_type, cmd_type = get_message(tipos['/lowstate']), get_message(tipos['/lowcmd'])

ts, qs, tc, qc, kp = [], [], [], [], []
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic == '/lowstate':
        m = deserialize_message(data, st_type)
        ts.append(stamp * 1e-9); qs.append([s.q for s in m.motor_state[:29]])
    elif topic == '/lowcmd':
        m = deserialize_message(data, cmd_type)
        tc.append(stamp * 1e-9); qc.append([c.q for c in m.motor_cmd[:29]])
        kp.append([c.kp for c in m.motor_cmd[:29]])

if not tc:
    sys.exit('No hay mensajes en /lowcmd. Comprueba que el ejemplo corre mientras grabas.')

ts, qs = np.array(ts), np.array(qs)
tc, qc, kp = np.array(tc), np.array(qc), np.array(kp)
t0 = tc[0]
sel = (tc - t0 >= t_ini) & (tc - t0 <= t_fin)
tc, qc, kp = tc[sel], qc[sel], kp[sel]
idx = np.searchsorted(ts, tc) - 1
ok = idx >= 0
tc, qc, kp, idx = tc[ok], qc[ok], kp[ok], idx[ok]
err = np.degrees(qc - qs[idx])
t = tc - t0

filas = []
for j in range(29):
    activo = kp[:, j] > 0
    if activo.sum() > 10:
        e = err[:, j]
        fuera = np.where(np.abs(e) > umbral)[0]
        if len(fuera) == 0:
            tset = 0.0
        elif fuera[-1] >= len(e) - 1:
            tset = float('nan')
        else:
            tset = t[fuera[-1] + 1] - t[0]
        filas.append((np.sqrt((e[activo]**2).mean()), np.abs(e[activo]).max(), tset, j))
filas.sort(reverse=True)

print(f"Ventana: {t_ini:.1f}-{min(t_fin, t[-1]+t0-t0):.1f} s | {len(t)} comandos | umbral de establecimiento: {umbral} deg")
print(f"{'articulacion':<16}{'RMS (deg)':>10}{'max (deg)':>11}{'t_estab (s)':>13}")
for rms, mx, tset, j in filas:
    ts_txt = 'no asienta' if np.isnan(tset) else f"{tset:.2f}"
    print(f"{NOMBRES[j]:<16}{rms:>10.2f}{mx:>11.2f}{ts_txt:>13}")


q_cmd = np.degrees(qc)
q_real = np.degrees(qs[idx])
fig, ax = plt.subplots(2, 2, figsize=(11, 7))
m = t <= t_ini + 2.5   # solo 2,5 s para ver bien las dos curvas
for k, nombre in enumerate(('L_ankle_pitch', 'L_ankle_roll')):
    j = NOMBRES.index(nombre)
    ax[0, k].plot(t[m], q_cmd[m, j], label='orden (/lowcmd)')
    ax[0, k].plot(t[m], q_real[m, j], label='real (/lowstate)')
    ax[0, k].set_title(nombre); ax[0, k].set_ylabel('posicion (deg)')
    ax[0, k].legend(); ax[0, k].grid(alpha=0.3)
    ax[1, k].plot(t, err[:, j], color='tab:red')
    ax[1, k].set_ylabel('error (deg)'); ax[1, k].set_xlabel('tiempo (s)')
    ax[1, k].grid(alpha=0.3)
fig.suptitle('Orden vs posicion real, tobillo izquierdo')
out = bag + '_seguimiento.png'
plt.savefig(out, dpi=130, bbox_inches='tight')
print('Grafica guardada en', out)
