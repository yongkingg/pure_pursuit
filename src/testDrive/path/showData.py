import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. 원본 데이터 읽기
data1 = pd.read_csv('./data.csv', header=None)

# 2. RANSAC 결과 읽기
ransac_data = pd.read_csv("ransacResult.csv", header=None)
model = ransac_data[ransac_data[0] == "Model"].iloc[0, 1:].reset_index(drop=True)

# 3. PolyRegression 결과 읽기
poly_data = pd.read_csv("polyResult.csv", header=None)
poly_model = poly_data.iloc[0].values  # 계수 읽기

# 4. 데이터 준비
# 원본 데이터 포인트
x_data = data1[0]
y_data = data1[1]

# RANSAC 모델 데이터 (이차 함수)
x_model = np.linspace(min(x_data), max(x_data), 500)
y_model_ransac = model[0] * x_model**2 + model[1] * x_model + model[2]

# PolyRegression 모델 데이터 (이차 함수로 가정)
y_model_poly = poly_model[0] * x_model**2 + poly_model[1] * x_model + poly_model[2]

# 5. 그래프 그리기
plt.figure(figsize=(10, 6))  # 그래프 크기 설정

# 원본 데이터 플롯 (파란 점)
plt.scatter(x_data, y_data, color='blue', label='Original Data', alpha=0.7)

# RANSAC 결과 플롯 (빨간 얇은 선)
plt.plot(x_model, y_model_ransac, color='red', linewidth=1, label="RANSAC Fit: y = {:.2f}x^2 + {:.2f}x + {:.2f}".format(*model))

# PolyRegression 결과 플롯 (초록색 얇은 선)
plt.plot(x_model, y_model_poly, color='green', linewidth=1, label="PolyRegression Fit: y = {:.2f}x^2 + {:.2f}x + {:.2f}".format(*poly_model))

# 6. 그래프 설정
plt.title('Comparison of Original Data, RANSAC, and PolyRegression Results')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)

# 7. 그래프 표시
plt.show()
