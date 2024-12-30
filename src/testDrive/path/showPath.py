import pandas as pd
import matplotlib.pyplot as plt

# 첫 번째 데이터 읽기 (UTM 좌표)
data1 = pd.read_csv('./K-City.csv', header=None, names=['utm_x', 'utm_y'])

# 두 번째 데이터 읽기 (Frenet 좌표)
data2 = pd.read_csv('./K-City-frenet.csv', header=None, names=['s', 'd'])

# 첫 번째 그래프: UTM 좌표
plt.figure(1)  # 첫 번째 창 생성
plt.plot(data1['utm_x'], data1['utm_y'], marker='o', linestyle='-', linewidth=0.5, label='UTM Path')
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

# 그래프 표시
plt.show()