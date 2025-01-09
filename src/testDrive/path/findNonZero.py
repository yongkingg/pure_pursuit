import pandas as pd

def find_nonzero_ranges(data):
    """
    0이 아닌 값의 연속적인 구간을 찾는 함수.
    
    Parameters:
        data (list or pandas.Series): 입력 데이터
    
    Returns:
        list of tuples: (start_index, end_index) 형태로 연속된 구간 반환
    """
    ranges = []
    start = None

    for i, value in enumerate(data):
        if value != 0:
            if start is None:  # 새로운 구간의 시작
                start = i
        else:
            if start is not None:  # 현재 구간 종료
                ranges.append((start, i - 1))
                start = None

    # 마지막 구간 처리
    if start is not None:
        ranges.append((start, len(data) - 1))

    return ranges


# 예제 데이터
data = pd.read_csv('./K-City-frenet.csv', header=None, names=['s', 'd'])

# 0이 아닌 값의 구간 찾기
nonzero_ranges = find_nonzero_ranges(data['d'])

# 결과 출력
print("Non-zero ranges:", nonzero_ranges)
