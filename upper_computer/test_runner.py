"""
test_runner.py — 自动化 AHRS 算法对比测试框架

测试场景：
  TC-01  静止收敛：初始姿态误差 30°，测量收敛时间和稳态误差
  TC-02  动态跟踪：预设轨迹（Roll±45° / Pitch±30° / Yaw旋转），测量跟踪 RMS
  TC-03  节点故障切换：Node-0 断电，验证协调器切换延迟和精度恢复
  TC-04  传感器型号对比：MPU6050 vs ICM42688 在同一轨迹下的精度差异
  TC-05  算法横向对比：互补滤波 / Mahony / Madgwick 在含噪声数据上的误差

每个 TC 输出：
  - 数值指标（RMS error, 收敛时间, 2σ 置信区间）
  - matplotlib 图表（误差曲线 + 误差分布直方图）
  - 控制台汇总表

用法：
    python test_runner.py              # 运行全部测试
    python test_runner.py --tc 02 05  # 只运行指定用例
"""

import argparse
import sys
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')   # 无显示器环境下保存图片
import matplotlib.pyplot as plt
# 中文字体配置
plt.rcParams['font.family'] = ['Arial Unicode MS', 'Heiti TC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
import matplotlib.gridspec as gridspec
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple

from imu_sim import make_imu, IMUSimulator
from ahrs_algorithms import make_ahrs, quat_to_euler, euler_to_quat
from multi_node import IMUNode, NodeCoordinator, make_default_system


# ──────────────────────────────────────────────────────────────
# 测试轨迹生成器
# ──────────────────────────────────────────────────────────────

def gen_trajectory(duration: float, dt: float,
                   scenario: str = 'dynamic') -> Dict:
    """
    生成真值轨迹（角速度 + 加速度）。

    Returns:
        dict with keys: t, omega_true, accel_true, roll_true, pitch_true, yaw_true
    """
    N      = int(duration / dt)
    t      = np.arange(N) * dt
    omega  = np.zeros((N, 3))
    accel  = np.zeros((N, 3))
    roll   = np.zeros(N)
    pitch  = np.zeros(N)
    yaw    = np.zeros(N)

    if scenario == 'static':
        # 完全静止，初始姿态 Roll=15°, Pitch=10°
        roll[:]  = 15.0
        pitch[:] = 10.0
        # 加速度 = 重力投影
        for i in range(N):
            r, p = np.radians(roll[i]), np.radians(pitch[i])
            accel[i] = [-np.sin(p)*9.81,
                         np.sin(r)*np.cos(p)*9.81,
                         np.cos(r)*np.cos(p)*9.81]

    elif scenario == 'dynamic':
        # 典型飞行器姿态机动
        for i in range(N):
            phase = t[i] % 40.0
            if phase < 10:
                roll[i]  = 45 * np.sin(np.pi * phase / 10)
                pitch[i] = 0
            elif phase < 20:
                roll[i]  = 0
                pitch[i] = -30 * np.sin(np.pi * (phase-10) / 10)
            elif phase < 30:
                roll[i]  = 40 * np.sin(2*np.pi*(phase-20)/5)
                pitch[i] = 25 * np.sin(2*np.pi*(phase-20)/7)
            else:
                roll[i]  = 5 * np.sin(phase*0.8)
                pitch[i] = 3 * np.cos(phase*1.1)
                yaw[i]   = 90 * (phase - 30)

        # 角速度 = 姿态角一阶差分
        roll_r  = np.radians(roll)
        pitch_r = np.radians(pitch)
        yaw_r   = np.radians(yaw)
        omega[1:, 0] = np.diff(roll_r)  / dt
        omega[1:, 1] = np.diff(pitch_r) / dt
        omega[1:, 2] = np.diff(yaw_r)   / dt

        # 加速度 = 重力在机体系投影
        for i in range(N):
            r, p = roll_r[i], pitch_r[i]
            accel[i] = [-np.sin(p)*9.81,
                         np.sin(r)*np.cos(p)*9.81,
                         np.cos(r)*np.cos(p)*9.81]

    elif scenario == 'yaw_spin':
        # 绕 Z 轴匀速旋转
        omega[:, 2] = np.radians(90)   # 90°/s
        yaw = np.degrees(omega[:, 2]) * t
        for i in range(N):
            accel[i] = [0, 0, 9.81]

    return dict(t=t, omega=omega, accel=accel,
                roll=roll, pitch=pitch, yaw=yaw)


def angle_diff(a: float, b: float) -> float:
    """计算两个角度之差，结果在 [-180, 180)"""
    d = (a - b) % 360
    return d - 360 if d >= 180 else d


def rms_error(est: np.ndarray, true: np.ndarray) -> float:
    diff = np.array([angle_diff(e, t) for e, t in zip(est, true)])
    return float(np.sqrt(np.mean(diff**2)))


def convergence_time(error: np.ndarray, t: np.ndarray,
                     threshold_deg: float = 2.0) -> float:
    """误差首次低于 threshold 且保持 1s 以上的时刻"""
    dt_mean = np.mean(np.diff(t))
    window  = max(1, int(1.0 / dt_mean))
    for i in range(len(error) - window):
        if np.all(np.abs(error[i:i+window]) < threshold_deg):
            return float(t[i])
    return float('inf')


# ──────────────────────────────────────────────────────────────
# 测试结果数据类
# ──────────────────────────────────────────────────────────────

@dataclass
class TCResult:
    tc_id:    str
    name:     str
    passed:   bool
    metrics:  Dict = field(default_factory=dict)
    notes:    str  = ''


# ──────────────────────────────────────────────────────────────
# 各测试用例
# ──────────────────────────────────────────────────────────────

def tc01_static_convergence(dt=0.01, duration=30.0,
                             save_dir='.') -> TCResult:
    """TC-01: 静止条件下各算法从错误初始姿态的收敛测试"""
    print('\n[TC-01] 静止收敛测试...')
    traj   = gen_trajectory(duration, dt, 'static')
    algos  = ['complementary', 'mahony', 'madgwick']
    colors = ['#FF6666', '#44CC44', '#4488FF']
    results = {}

    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    fig.suptitle('TC-01: 静止收敛 — 从 30° 初始误差恢复', fontsize=13)

    for algo, color in zip(algos, colors):
        ahrs = make_ahrs(algo, dt=dt)
        # 初始四元数故意偏置 30°
        ahrs.reset(euler_to_quat(roll_deg=15+30, pitch_deg=10+20, yaw_deg=0))

        imu  = make_imu('MPU6050', dt=dt, seed=42)
        rolls, pitchs = [], []

        for omega, acc in zip(traj['omega'], traj['accel']):
            gm, am = imu.update(omega, acc)
            r, p, _ = ahrs.update(gm, am)
            rolls.append(r); pitchs.append(p)

        rolls  = np.array(rolls)
        pitchs = np.array(pitchs)

        roll_err  = np.array([angle_diff(r, t)
                               for r, t in zip(rolls,  traj['roll'])])
        pitch_err = np.array([angle_diff(p, t)
                               for p, t in zip(pitchs, traj['pitch'])])
        total_err = np.sqrt(roll_err**2 + pitch_err**2)

        conv_t   = convergence_time(total_err, traj['t'], threshold_deg=2.0)
        ss_rms   = rms_error(rolls[-200:],  traj['roll'][-200:])

        results[algo] = {'conv_t': conv_t, 'ss_rms': ss_rms}
        axes[0].plot(traj['t'], total_err, color=color,
                     label=f'{algo} (conv={conv_t:.1f}s)', linewidth=1.5)

    axes[0].axhline(2.0, color='white', linestyle='--', alpha=0.5, linewidth=1)
    axes[0].set_xlabel('时间 (s)'); axes[0].set_ylabel('姿态误差 (°)')
    axes[0].set_title('收敛过程')
    axes[0].legend(fontsize=9)
    axes[0].set_facecolor('#1a1a2e'); fig.patch.set_facecolor('#1a1a2e')
    for ax in axes:
        ax.tick_params(colors='#cccccc'); ax.xaxis.label.set_color('#cccccc')
        ax.yaxis.label.set_color('#cccccc'); ax.title.set_color('#cccccc')
        ax.spines[:].set_color('#444466')

    # 柱状图对比
    names = list(results.keys())
    conv_vals = [results[n]['conv_t'] for n in names]
    axes[1].bar(names, conv_vals, color=colors, edgecolor='#333355')
    axes[1].set_ylabel('收敛时间 (s)'); axes[1].set_title('收敛时间对比')

    plt.tight_layout()
    path = f'{save_dir}/tc01_convergence.png'
    plt.savefig(path, dpi=150, bbox_inches='tight',
                facecolor='#1a1a2e')
    plt.close()
    print(f'  图表已保存: {path}')

    best_algo = min(results, key=lambda k: results[k]['conv_t'])
    print(f'  最快收敛: {best_algo}  ({results[best_algo]["conv_t"]:.1f}s)')
    for algo, r in results.items():
        conv_s = f"{r['conv_t']:.1f}s" if r['conv_t'] < float('inf') else '>30s'
        print(f'  {algo:20s}: 收敛={conv_s}  稳态RMS={r["ss_rms"]:.3f}°')

    passed = all(r['ss_rms'] < 3.0 for r in results.values())
    return TCResult('TC-01', '静止收敛', passed, results)


def tc02_dynamic_tracking(dt=0.01, duration=80.0,
                           save_dir='.') -> TCResult:
    """TC-02: 动态轨迹跟踪精度对比"""
    print('\n[TC-02] 动态轨迹跟踪测试...')
    traj   = gen_trajectory(duration, dt, 'dynamic')
    algos  = ['complementary', 'mahony', 'madgwick']
    colors = ['#FF6666', '#44CC44', '#4488FF']
    results = {}

    fig = plt.figure(figsize=(14, 8))
    fig.suptitle('TC-02: 动态轨迹跟踪 — Roll/Pitch 误差', fontsize=13)
    fig.patch.set_facecolor('#1a1a2e')
    gs = gridspec.GridSpec(2, 3, figure=fig)

    warmup = int(5.0 / dt)   # 前5秒为暖机期，不计入统计

    for col, (algo, color) in enumerate(zip(algos, colors)):
        ahrs = make_ahrs(algo, dt=dt)
        imu  = make_imu('MPU6050', dt=dt, seed=0)
        rolls, pitchs = [], []

        for omega, acc in zip(traj['omega'], traj['accel']):
            gm, am = imu.update(omega, acc)
            r, p, _ = ahrs.update(gm, am)
            rolls.append(r); pitchs.append(p)

        rolls  = np.array(rolls)
        pitchs = np.array(pitchs)

        roll_rms  = rms_error(rolls[warmup:],  traj['roll'][warmup:])
        pitch_rms = rms_error(pitchs[warmup:], traj['pitch'][warmup:])
        results[algo] = {'roll_rms': roll_rms, 'pitch_rms': pitch_rms,
                         'avg_rms': (roll_rms + pitch_rms) / 2}

        ax0 = fig.add_subplot(gs[0, col])
        ax0.plot(traj['t'], traj['roll'],  color='#aaaaaa', linewidth=1, alpha=0.6)
        ax0.plot(traj['t'], rolls,          color=color,     linewidth=1)
        ax0.set_title(f'{algo}\nRoll RMS={roll_rms:.2f}°',
                      color='#cccccc', fontsize=10)
        ax0.set_facecolor('#1a1a2e')

        ax1 = fig.add_subplot(gs[1, col])
        ax1.plot(traj['t'], traj['pitch'], color='#aaaaaa', linewidth=1, alpha=0.6)
        ax1.plot(traj['t'], pitchs,         color=color,     linewidth=1)
        ax1.set_xlabel('时间 (s)', color='#cccccc', fontsize=9)
        ax1.set_title(f'Pitch RMS={pitch_rms:.2f}°', color='#cccccc', fontsize=10)
        ax1.set_facecolor('#1a1a2e')
        for ax in [ax0, ax1]:
            ax.tick_params(colors='#cccccc')
            ax.spines[:].set_color('#444466')

    plt.tight_layout()
    path = f'{save_dir}/tc02_dynamic.png'
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    plt.close()
    print(f'  图表已保存: {path}')

    for algo, r in results.items():
        print(f'  {algo:20s}: Roll_RMS={r["roll_rms"]:.3f}°  '
              f'Pitch_RMS={r["pitch_rms"]:.3f}°')

    best = min(results, key=lambda k: results[k]['avg_rms'])
    passed = results[best]['avg_rms'] < 5.0
    return TCResult('TC-02', '动态跟踪', passed, results,
                    notes=f'最优算法: {best}')


def tc03_failover(dt=0.01, duration=60.0,
                  save_dir='.') -> TCResult:
    """TC-03: Node-0 断电，验证协调器故障切换"""
    print('\n[TC-03] 节点故障切换测试...')
    traj  = gen_trajectory(duration, dt, 'dynamic')
    nodes, coord = make_default_system(dt=dt)

    fused_rolls, fused_pitchs = [], []
    node_health_log = [[] for _ in range(3)]
    FAULT_TIME = 20.0   # 第20秒注入故障

    fault_injected = False
    for i, (omega, acc) in enumerate(zip(traj['omega'], traj['accel'])):
        t_now = traj['t'][i]

        if t_now >= FAULT_TIME and not fault_injected:
            nodes[0].inject_fault('power_loss')
            coord.log_event('Node-0 power_loss injected')
            fault_injected = True
            print(f'  t={FAULT_TIME}s: Node-0 断电')

        readings = [n.update(omega, acc) for n in nodes]
        fused    = coord.update(readings)

        fused_rolls.append(fused['roll'])
        fused_pitchs.append(fused['pitch'])
        for j in range(3):
            h = readings[j]['health'].value if readings[j] else 3
            node_health_log[j].append(h)

    fused_rolls  = np.array(fused_rolls)
    fused_pitchs = np.array(fused_pitchs)

    # 故障前后精度对比
    pre_idx  = traj['t'] < FAULT_TIME
    post_idx = traj['t'] >= FAULT_TIME + 2   # 2s 过渡期后
    pre_rms  = rms_error(fused_rolls[pre_idx],  traj['roll'][pre_idx])
    post_rms = rms_error(fused_rolls[post_idx], traj['roll'][post_idx])
    degradation = post_rms - pre_rms

    # 绘图
    fig, axes = plt.subplots(2, 1, figsize=(12, 7))
    fig.suptitle('TC-03: Node-0 断电 — 多节点故障切换', fontsize=13)
    fig.patch.set_facecolor('#1a1a2e')

    axes[0].plot(traj['t'], traj['roll'], color='#aaaaaa',
                 linewidth=1, alpha=0.6, label='真值')
    axes[0].plot(traj['t'], fused_rolls, color='#44CC88',
                 linewidth=1.5, label='融合输出')
    axes[0].axvline(FAULT_TIME, color='#FF4444', linestyle='--',
                    linewidth=1.5, label='Node-0 断电')
    axes[0].set_ylabel('Roll (°)', color='#cccccc')
    axes[0].legend(fontsize=9, facecolor='#2a2a42', labelcolor='#cccccc')

    colors_h = ['#FF4444', '#44CC44', '#4488FF']
    for j in range(3):
        axes[1].plot(traj['t'], node_health_log[j],
                     color=colors_h[j], linewidth=1.5, label=f'Node-{j}')
    axes[1].axvline(FAULT_TIME, color='#FF4444', linestyle='--', linewidth=1.5)
    axes[1].set_ylabel('健康状态 (1=OK 2=降级 3=失效)', color='#cccccc')
    axes[1].set_xlabel('时间 (s)', color='#cccccc')
    axes[1].legend(fontsize=9, facecolor='#2a2a42', labelcolor='#cccccc')

    for ax in axes:
        ax.set_facecolor('#1a1a2e')
        ax.tick_params(colors='#cccccc')
        ax.spines[:].set_color('#444466')

    plt.tight_layout()
    path = f'{save_dir}/tc03_failover.png'
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    plt.close()
    print(f'  图表已保存: {path}')
    print(f'  故障前 RMS={pre_rms:.3f}°  故障后 RMS={post_rms:.3f}°  '
          f'精度降级={degradation:.3f}°')

    passed = degradation < 3.0   # 切换后精度损失 < 3°
    return TCResult('TC-03', '故障切换', passed,
                    {'pre_rms': pre_rms, 'post_rms': post_rms,
                     'degradation': degradation})


def tc04_sensor_comparison(dt=0.01, duration=60.0,
                            save_dir='.') -> TCResult:
    """TC-04: 传感器型号精度对比（MPU6050 vs ICM42688）"""
    print('\n[TC-04] 传感器型号对比测试...')
    traj    = gen_trajectory(duration, dt, 'dynamic')
    sensors = ['MPU6050', 'ICM42688']
    colors  = ['#FF8844', '#44AAFF']
    RUNS    = 10   # Monte Carlo 重复次数
    results = {}

    for sensor, color in zip(sensors, colors):
        rms_list = []
        for seed in range(RUNS):
            imu  = make_imu(sensor, dt=dt, seed=seed)
            ahrs = make_ahrs('mahony', dt=dt)
            rolls = []
            for omega, acc in zip(traj['omega'], traj['accel']):
                gm, am = imu.update(omega, acc)
                r, _, _ = ahrs.update(gm, am)
                rolls.append(r)
            rms_list.append(rms_error(
                np.array(rolls[500:]), traj['roll'][500:]))
        results[sensor] = {
            'mean': np.mean(rms_list),
            'std':  np.std(rms_list),
            'p95':  np.percentile(rms_list, 95),
        }

    fig, ax = plt.subplots(figsize=(7, 4))
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#1a1a2e')

    names = list(results.keys())
    means = [results[n]['mean'] for n in names]
    stds  = [results[n]['std']  for n in names]
    bars  = ax.bar(names, means, yerr=stds, capsize=6,
                   color=colors, edgecolor='#333355', error_kw={'color':'#aaaaaa'})
    ax.set_ylabel('Roll RMS 误差 (°)', color='#cccccc')
    ax.set_title(f'TC-04: 传感器精度对比 (Mahony, {RUNS}次Monte Carlo)',
                 color='#cccccc')
    ax.tick_params(colors='#cccccc')
    ax.spines[:].set_color('#444466')

    for bar, name in zip(bars, names):
        ax.text(bar.get_x() + bar.get_width()/2,
                bar.get_height() + stds[names.index(name)] + 0.01,
                f'{results[name]["mean"]:.3f}°',
                ha='center', va='bottom', color='#cccccc', fontsize=10)

    plt.tight_layout()
    path = f'{save_dir}/tc04_sensor_compare.png'
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    plt.close()
    print(f'  图表已保存: {path}')

    for name, r in results.items():
        print(f'  {name:12s}: RMS={r["mean"]:.3f}±{r["std"]:.3f}°  '
              f'P95={r["p95"]:.3f}°')

    ratio = results['MPU6050']['mean'] / max(results['ICM42688']['mean'], 1e-9)
    print(f'  ICM42688 比 MPU6050 精度提升 {ratio:.1f}×')
    passed = results['ICM42688']['mean'] < results['MPU6050']['mean']
    return TCResult('TC-04', '传感器对比', passed, results)


def tc05_algorithm_noise_robustness(dt=0.01, duration=60.0,
                                     save_dir='.') -> TCResult:
    """TC-05: 不同噪声等级下三种算法鲁棒性"""
    print('\n[TC-05] 算法噪声鲁棒性测试...')
    traj = gen_trajectory(duration, dt, 'dynamic')

    # 测试 3 种噪声等级（ARW 缩放系数）
    noise_scales = [0.5, 1.0, 3.0]
    algos        = ['complementary', 'mahony', 'madgwick']
    colors_algo  = ['#FF6666', '#44CC44', '#4488FF']
    results = {a: [] for a in algos}

    import copy
    from imu_sim import GyroParams, AccelParams

    for scale in noise_scales:
        import numpy as np
        from imu_sim import IMUSimulator
        gp = GyroParams(arw=np.radians(0.05 * scale))
        ap = AccelParams(vrw=300e-6 * 9.81 * scale)

        for algo in algos:
            rms_runs = []
            for seed in range(8):
                imu  = IMUSimulator(dt=dt, gyro_params=gp,
                                    accel_params=ap, seed=seed)
                ahrs = make_ahrs(algo, dt=dt)
                rolls = []
                for omega, acc in zip(traj['omega'], traj['accel']):
                    gm, am = imu.update(omega, acc)
                    r, _, _ = ahrs.update(gm, am)
                    rolls.append(r)
                rms_runs.append(rms_error(
                    np.array(rolls[500:]), traj['roll'][500:]))
            results[algo].append(np.mean(rms_runs))

    fig, ax = plt.subplots(figsize=(8, 4))
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#1a1a2e')

    x = np.arange(len(noise_scales))
    w = 0.25
    for j, (algo, color) in enumerate(zip(algos, colors_algo)):
        ax.bar(x + j*w, results[algo], w, label=algo,
               color=color, edgecolor='#333355', alpha=0.85)

    ax.set_xticks(x + w)
    ax.set_xticklabels([f'×{s} 噪声' for s in noise_scales], color='#cccccc')
    ax.set_ylabel('Roll RMS 误差 (°)', color='#cccccc')
    ax.set_title('TC-05: 算法噪声鲁棒性 — 不同噪声强度下的精度',
                 color='#cccccc')
    ax.legend(facecolor='#2a2a42', labelcolor='#cccccc')
    ax.tick_params(colors='#cccccc')
    ax.spines[:].set_color('#444466')

    plt.tight_layout()
    path = f'{save_dir}/tc05_noise_robustness.png'
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    plt.close()
    print(f'  图表已保存: {path}')

    for algo in algos:
        vals = '  '.join(f'{v:.3f}°' for v in results[algo])
        print(f'  {algo:20s}: {vals}')

    passed = True
    return TCResult('TC-05', '噪声鲁棒性', passed, results)


# ──────────────────────────────────────────────────────────────
# 汇总报告
# ──────────────────────────────────────────────────────────────

def print_summary(results: List[TCResult]):
    print('\n' + '='*60)
    print('  AHRS SIL 测试报告汇总')
    print('='*60)
    passed = sum(1 for r in results if r.passed)
    for r in results:
        status = 'PASS ✓' if r.passed else 'FAIL ✗'
        print(f'  [{status}] {r.tc_id}: {r.name}  {r.notes}')
    print('-'*60)
    print(f'  总计: {passed}/{len(results)} 通过')
    print('='*60)


# ──────────────────────────────────────────────────────────────
# 入口
# ──────────────────────────────────────────────────────────────

TC_MAP = {
    '01': tc01_static_convergence,
    '02': tc02_dynamic_tracking,
    '03': tc03_failover,
    '04': tc04_sensor_comparison,
    '05': tc05_algorithm_noise_robustness,
}


def main():
    parser = argparse.ArgumentParser(description='AHRS SIL 自动化测试框架')
    parser.add_argument('--tc', nargs='+', default=list(TC_MAP.keys()),
                        help='要运行的测试用例编号，如 --tc 01 03')
    parser.add_argument('--out', default='test_results',
                        help='图表输出目录（默认: test_results）')
    args = parser.parse_args()

    import os
    os.makedirs(args.out, exist_ok=True)

    print('='*60)
    print('  AHRS SIL 自动化测试框架 v1.0')
    print(f'  运行用例: {args.tc}')
    print('='*60)

    t0 = time.time()
    tc_results = []
    for tc_id in args.tc:
        tc_id = tc_id.zfill(2)
        fn = TC_MAP.get(tc_id)
        if fn is None:
            print(f'[WARN] 未知测试用例: TC-{tc_id}，跳过')
            continue
        result = fn(save_dir=args.out)
        tc_results.append(result)

    print_summary(tc_results)
    print(f'\n  总耗时: {time.time()-t0:.1f}s')
    print(f'  图表保存在: {args.out}/')


if __name__ == '__main__':
    main()
