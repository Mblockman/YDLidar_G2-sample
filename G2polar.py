import serial
import binascii
import G2Lidar  
import matplotlib.pyplot as plt
import math
import time

# 모든 기존의 plt 창을 닫습니다.
plt.close('all')

fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='polar')
ax.set_title('lidar (exit: Key Q)',fontsize=18)

# 키 이벤트 핸들러 함수
def on_key(event):
    if event.key == 'q':
        lidar.stop()
        plt.close(fig)  # 플롯 창 닫기

plt.gcf().canvas.mpl_connect('key_press_event', on_key)

lidar = G2Lidar.G2Lidar()

if lidar.begin('COM5'):
    print("Lidar connected")
    print("Start Scan")
    lidar.startScan()

lines = list()
angles = list()
distances = list()

# 데이터 업데이트 및 플롯 갱신 함수
def main():
    global lines, angles, distances
    count = 0
    scan_data = lidar.waitScanData()
    scan_data = None    
    while plt.fignum_exists(fig.number):  # 플롯 창이 열려 있는 동안 루프 실행
        if(count % 40 == 39):
            try:
                if('line' in locals()):
                    line.remove()
                line = ax.scatter(angles, distances, c="blue", s=2)
            except:     
                pass
            ax.set_theta_offset(math.pi / 2)
            plt.pause(0.01)
            angles.clear()
            distances.clear()
            count = 0   

        scan_data = lidar.scanData()
        if scan_data is not None:
            if lidar.process_lidar_sumCheck(scan_data):
                angle, distance = lidar.process_lidar_data(scan_data,True)
                if angle is not None and distance is not None:
                    #print(f"angle: {[math.radians(a) for a in angle]}, distance: {distance}")
                    angles.extend([math.radians(a) for a in angle])
                    distances.extend(distance)
        count += 1

    lidar.stop()                            


if __name__ == "__main__":
    main()
