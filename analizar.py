import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
msg_type = get_message(types['/lowstate'])

t, rpy, gyro, dq, tau = [], [], [], [], []
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic != '/lowstate':
        continue
    m = deserialize_message(data, msg_type)
    t.append(stamp * 1e-9)
    rpy.append(list(m.imu_state.rpy))
    gyro.append(list(m.imu_state.gyroscope))
    dq.append([s.dq for s in m.motor_state[:29]])
    tau.append([s.tau_est for s in m.motor_state[:29]])

t = np.array(t); t -= t[0]
rpy = np.degrees(np.array(rpy)); gyro = np.array(gyro)
dq = np.array(dq); tau = np.array(tau)
power = np.sum(np.abs(tau * dq), axis=1)

print(f"Muestras: {len(t)}  Duracion: {t[-1]:.2f} s  Frecuencia media: {1/np.diff(t).mean():.1f} Hz")
print(f"Roll max: {np.abs(rpy[:,0]).max():.1f} deg   Pitch max: {np.abs(rpy[:,1]).max():.1f} deg")
print(f"Gyro RMS: {np.sqrt((gyro**2).sum(axis=1).mean()):.3f} rad/s")
print(f"Potencia mecanica media (aprox): {power.mean():.2f} W")
caida = (np.abs(rpy[:,0]) > 45) | (np.abs(rpy[:,1]) > 45)
print(f"Muestras con inclinacion > 45 deg: {caida.sum()}")

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, sharex=True)
    ax[0].plot(t, rpy[:,0], label='roll'); ax[0].plot(t, rpy[:,1], label='pitch')
    ax[0].set_ylabel('deg'); ax[0].legend()
    ax[1].plot(t, power); ax[1].set_ylabel('W'); ax[1].set_xlabel('s')
    plt.savefig('/root/g1_ws/metricas.png', dpi=120)
    print('Grafica guardada en g1_ws/metricas.png')
except ImportError:
    print('Instala matplotlib para la grafica: pip install matplotlib')
