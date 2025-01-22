import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. 원본 데이터 읽기
# data1 = pd.read_csv('./sliced_data.csv', header=None)  # 원본 경로 데이터
data1 = pd.read_csv('./data.csv', header=None, delim_whitespace=True)
data1.columns = ['east', 'north', 'up']

# 2. RANSAC 결과 읽기
ransac_data = pd.read_csv("ransacResult.csv", header=None)
ransac_model_row = ransac_data[ransac_data[0] == "Model"].iloc[0, 1:]  # "Model" 행의 두 번째 열부터 읽기
model = ransac_model_row.values.astype(float)  # float 배열로 변환

# 3. Left, Right 데이터 읽기
left_data = pd.read_csv('./left.csv', header=None)  # 왼쪽 줄 데이터
left_data.columns = ['east', 'north', 'up']

right_data = pd.read_csv('./right.csv', header=None)  # 오른쪽 줄 데이터
right_data.columns = ['east', 'north', 'up']

# 4. 데이터 준비
x_data = data1['east']  # 원본 데이터 x 값
y_data = data1['north']  # 원본 데이터 y 값

x_model = np.linspace(min(x_data), max(x_data), 500)  # RANSAC 모델의 x 값
y_model_ransac = model[0] * x_model**2 + model[1] * x_model + model[2]  # RANSAC 모델의 y 값

# Left, Right 데이터 준비
x_left = left_data['east']
y_left = left_data['north']

x_right = right_data['east']
y_right = right_data['north']

# 5. 그래프 그리기
plt.figure(figsize=(12, 8))  # 그래프 크기 설정

# 원본 데이터 (파란 점)
plt.plot(x_data, y_data, color='blue', linestyle='--', label='Original Data')

# 왼쪽 줄 데이터 (초록색 점선)
plt.plot(x_left, y_left, color='green', linestyle='--', linewidth=1.5, label='Left Line')

# 오른쪽 줄 데이터 (주황색 점선)
plt.plot(x_right, y_right, color='orange', linestyle='--', linewidth=1.5, label='Right Line')


tmpPoint = pd.read_csv('./tmpPoint.csv', header=None, delim_whitespace=True)
tmpPoint.columns = ['east', 'north', 'up']
tmp_x = tmpPoint['east']
tmp_y = tmpPoint['north']

plt.scatter(tmp_x, tmp_y, color="red", label="Scatter Points", s=10)

# 6. 그래프 설정
plt.title('Original Data, RANSAC Result, and Left/Right Lines')
plt.xlabel('East')
plt.ylabel('North')
plt.legend()
plt.grid(True)

# 7. 그래프 표시
plt.show()
