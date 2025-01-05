import pandas as pd
import matplotlib.pyplot as plt

data1 = pd.read_csv('./K-City.csv', header=None, names=['utm_x', 'utm_y'])
data2 = pd.read_csv('./K-City-frenet.csv', header=None, names=['s', 'd'])
data3 = pd.read_csv('./K-City-reconstructed.csv', header=None, names=['utm_x', 'utm_y'])

# 여러 구간 정의 (시작, 종료 인덱스)
highlight_sections = [
    (537, 545),  # 첫 번째 구간
    (1114, 1116),  # 두 번째 구간
]

# 첫 번째 그래프: UTM 좌표
plt.figure(1)  # 첫 번째 창 생성

# 전체 경로 플롯 (파란색)
plt.plot(data1['utm_x'], data1['utm_y'], color='blue', linestyle='-', linewidth=0.5, label='UTM Path')

# 각 구간 강조 (빨간색)
for start, end in highlight_sections:
    highlight_x = data1['utm_x'][start:end+1]
    highlight_y = data1['utm_y'][start:end+1]
    plt.plot(highlight_x, highlight_y, color='red', linestyle='-', linewidth=1.5, label=f'Highlight {start}-{end}')

plt.title('UTM Coordinates')
plt.xlabel('UTM X (meters)')
plt.ylabel('UTM Y (meters)')
plt.legend()
plt.grid(True)

# 두 번째 그래프: Frenet 좌표
plt.figure(2)  # 두 번째 창 생성
plt.plot(data2['s'], data2['d'], marker='x', linestyle='--', linewidth=0.5, label='Frenet Path')

# Frenet 좌표에 대한 설정
plt.title('Frenet Coordinates')
plt.xlabel('S (meters)')  # S: 곡선의 누적 거리
plt.ylabel('D (meters)')  # D: 곡선 기준의 법선 거리

# 축 범위 설정 (예: 데이터 범위를 기반으로 자동 또는 수동 설정 가능)
plt.xlim(data2['s'].min() - 5, data2['s'].max() + 5)  # S 범위 (여유 공간 포함)
plt.ylim(data2['d'].min() - 1, data2['d'].max() + 1)  # D 범위 (여유 공간 포함)

plt.legend()
plt.grid(True)


plt.figure(3)
plt.plot(data1['utm_x'], data1['utm_y'], color='blue', linestyle='-', linewidth=0.5, label='UTM Path')
plt.title('UTM Coordinates (re-Constructed)')
plt.xlabel('UTM X (meters)')
plt.ylabel('UTM Y (meters)')
plt.legend()
plt.grid(True)

# 그래프 표시
plt.show()
