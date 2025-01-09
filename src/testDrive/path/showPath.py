import pandas as pd
import matplotlib.pyplot as plt

data1 = pd.read_csv('./K-City.csv', header=None, names=['utm_x', 'utm_y'])
data2 = pd.read_csv('./K-City-frenet.csv', header=None, names=['s', 'd'])
data3 = pd.read_csv('./K-City-reconstructed.csv', header=None, names=['utm_x', 'utm_y'])

# 여러 구간 정의 (시작, 종료 인덱스)
highlight_sections = [
(306, 306), (316, 317), (319, 319), (323, 323), (326, 326), (328, 328), (330, 331), (336, 338), (340, 342), (346, 347), (349, 352), (355, 357), (359, 361), (365, 365), (368, 368), (371, 371), (387, 388), (393, 393), (398, 398), (407, 407), (411, 411), (468, 468), (535, 543), (705, 705), (719, 719), (733, 733), (789, 789), (813, 814), (825, 825), (829, 831), (833, 865), (867, 874), (876, 879), (882, 884), (886, 886), (888, 898), (900, 902), (904, 906), (908, 912), (915, 918), (920, 924), (926, 931), (933, 935), (938, 938), (940, 941), (943, 944), (946, 946), (951, 952), (954, 956), (958, 963), (966, 967), (969, 976), (978, 981), (983, 983), (990, 991), (993, 995), (997, 997), (1000, 1000), (1002, 1006), (1008, 1009), (1011, 1013), (1015, 1021), (1025, 1027), (1029, 1030), (1032, 1032), (1036, 1039), (1041, 1044), (1053, 1053), (1056, 1056), (1060, 1061), (1063, 1066), (1071, 1072), (1082, 1084), (1087, 1091), (1093, 1095), (1097, 1099), (1102, 1102), (1104, 1104), (1107, 1107), (1109, 1109), (1111, 1115), (1119, 1119), (1122, 1122), (1129, 1129), (1131, 1132), (1134, 1134), (1136, 1136), (1138, 1139), (1141, 1142), (1144, 1147), (1151, 1151), (1157, 1157), (1159, 1159), (1168, 1168), (1170, 1174), (1176, 1179), (1181, 1183), (1185, 1209), (1211, 1218), (1220, 1222), (1224, 1227), (1229, 1229), (1231, 1234), (1236, 1247), (1249, 1256), (1258, 1261), (1263, 1263), (1265, 1265), (1267, 1268), (1270, 1275), (1278, 1281), (1283, 1295), (1297, 1308), (1310, 1316), (1318, 1333), (1335, 1338), (1341, 1347), (1349, 1349), (1351, 1353), (1355, 1355), (1358, 1359), (1361, 1368), (1370, 1372), (1374, 1377), (1379, 1380), (1382, 1403), (1600, 1601), (1603, 1605), (1608, 1608), (1610, 1623), (1625, 1625), (1629, 1634), (1638, 1639), (1642, 1643), (1646, 1646), (1648, 1648), (1650, 1650), (1653, 1653), (1656, 1656), (1662, 1662), (1666, 1666), (1675, 1676), (1685, 1685), (1688, 1688), (1693, 1694), (1697, 1697), (1702, 1702), (1705, 1706), (1710, 1710), (1712, 1713), (1715, 1715), (1718, 1718), (1728, 1729), (1731, 1731), (1738, 1738), (1742, 1742), (1744, 1745), (1747, 1747), (1751, 1751), (1755, 1755), (1758, 1758), (1761, 1761), (1766, 1766), (1774, 1788), (1790, 1792), (1794, 1803), (1806, 1816), (1819, 1821), (1823, 1828), (1830, 1831), (1834, 1835), (1837, 1840), (1842, 1844), (1846, 1847), (1851, 1852), (1854, 1855), (1857, 1859), (1861, 1862), (1865, 1866), (1868, 1868), (1870, 1870), (1872, 1874), (1876, 1876), (1881, 1881), (1883, 1885), (1888, 1888), (1891, 1892), (1895, 1896), (1899, 1900), (1902, 1905), (1907, 1912), (1914, 2180), (2182, 2182), (2185, 2188), (2191, 2194), (2196, 2205), (2208, 2210), (2212, 2212), (2214, 2217), (2220, 2220), (2223, 2224), (2226, 2229), (2231, 2232), (2234, 2234), (2236, 2236), (2240, 2240), (2242, 2242), (2244, 2244), (2250, 2252), (2254, 2254), (2256, 2257), (2259, 2261), (2264, 2266), (2268, 2268), (2270, 2270), (2273, 2273), (2280, 2280), (2282, 2282), (2285, 2285), (2288, 2298), (2300, 2313), (2315, 2315), (2317, 2339), (2617, 2617), (2626, 2626), (2636, 2637), (2639, 2639), (2641, 2644), (2646, 2686), (2688, 2690), (2692, 2695), (2697, 2697), (2699, 2699), (2701, 2704), (2706, 2707), (2709, 2711), (2714, 2717), (2721, 2721), (2723, 2723), (2725, 2725), (2727, 2727), (2729, 2729), (2731, 2734), (2736, 2737), (2739, 2739), (2741, 2743), (2745, 2745), (2747, 2747), (2749, 2751), (2756, 2757), (2759, 2762), (2765, 2767), (2771, 2772), (2777, 2778), (2781, 2781), (2785, 2787), (2789, 2791), (2795, 2797), (2799, 2799), (2803, 2804), (2806, 2806), (2809, 2810), (2815, 2816), (2818, 2819), (2822, 2822), (2824, 2824), (2830, 2830), (2833, 2833), (2835, 2835), (2837, 2838), (2841, 2841), (2843, 2843), (2846, 2846), (2850, 2850), (2853, 2856), (2858, 2859), (2865, 2865), (2868, 2869), (2872, 2872), (2874, 2874), (2877, 2877), (2879, 2880), (2884, 2884), (2896, 2896), (2900, 2900), (2903, 2903), (2905, 2905), (2914, 2914), (2926, 2928), (2931, 2932), (2938, 2938), (2942, 2942), (2946, 2946), (2952, 2952), (2954, 2954), (2958, 2959), (2963, 2963), (2965, 2965), (2969, 2969), (2971, 2971), (2974, 2974), (2987, 2987), (2992, 2992), (2995, 2995), (3006, 3006), (3015, 3015), (3021, 3021), (3024, 3024), (3028, 3029), (3031, 3031), (3047, 3047), (3049, 3049), (3055, 3055), (3085, 3113), (3115, 3131), (3133, 3135), (3137, 3137), (3142, 3142), (3158, 3160), (3163, 3163), (3167, 3169), (3171, 3171), (3174, 3174), (3178, 3178), (3180, 3180), (3182, 3182), (3184, 3184), (3187, 3189), (3191, 3192)
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
