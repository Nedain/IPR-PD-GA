from controller import Supervisor
import csv
import numpy as np
import matplotlib.pyplot as plt
import datetime

# TIME_STEP = 32
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

#Titik posisi dan rotasi awal robot pada world
sbr_node = robot.getFromDef('SBR')
translation_field = sbr_node.getField('translation')
rotation_field = sbr_node.getField("rotation")

#Variabel IMU
inemu = robot.getDevice('InertialUnit')
inemu.enable(timestep)
initial_pitch_angle = inemu.getRollPitchYaw()[1]

#Variabel Motor
left_motor = robot.getDevice('Motor_L')
right_motor = robot.getDevice('Motor_R')
left_motor.setPosition(float('+inf'))
left_motor.setVelocity(0.0)
right_motor.setPosition(float('+inf'))
right_motor.setVelocity(0.0)
maxSpeed = min(right_motor.getMaxVelocity(), left_motor.getMaxVelocity())

#Variabel Sensor Posisi (Encoder)
left_encoder = robot.getDevice('Encoder_L')
left_encoder.enable(timestep)
right_encoder = robot.getDevice('Encoder_R')
right_encoder.enable(timestep)
position_values = [0.0,0.0]
position_values[0] = left_encoder.getValue()
position_values[1] = right_encoder.getValue()
last_position_values = [0.0,0.0]

#Variabel PD ENCODER
#Kp_enc = 0.06 #0.06
#Kd_enc = 21

#Variabel PD ENCODER
Kp_enc = 0.06 #0.06
Kd_enc = 21

#Variabel PD IMU
KP = 173.4 #175.0***
KD = 1.9 #3.0***

#Variabel error
error_enc_new = 0.0
error_imu_new = 0.0
t = []
jumlah = []
error_imu_t = []
posisi_t = []
error_imu_abs = 0.0
error_enc_abs = 0.0
error_tot_abs = 0.0
imu_abs = []
enc_abs = []
abs_tot = []
imu_abs_ma = []
enc_abs_ma = []
IAE_enc = 0.0
IAE_imu = 0.0
IAE_tot = 0.0

#Variabel waktu
time_awal = robot.getTime() + 0.032
last_time = 0.0
time_step = 0

#-----------------------------SIMULASI BERJALAN----------------------------
while robot.step(timestep) != -1:
    current_time = robot.getTime()
    # print(f"{position_values[0]}")
        
    #PD Encoder
    last_position_values[0] = left_encoder.getValue()
    print(last_position_values[0])
    error_enc = position_values[0] - last_position_values[0]
    # error_enc = last_position_values[0]
    P_enc = Kp_enc * error_enc
    D_enc = Kd_enc * (error_enc - error_enc_new) / timestep
    
    #PD IMU
    current_pitch_angle = inemu.getRollPitchYaw()[1]
        
    error_imu = current_pitch_angle - initial_pitch_angle
    P_imu = KP * error_imu
    D_imu = KD * ((error_imu - error_imu_new) / timestep)
    
    #Mengatur kecepatan robot sesuai kontrol PD
    speed = (P_imu + D_imu) - (P_enc + D_enc)
    if speed > maxSpeed:
        speed = maxSpeed
    elif speed < -maxSpeed:
        speed = -maxSpeed
    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed)
    
    #Menghitung nilai error absolut dan integral absolut error
    error_enc_abs = np.abs(error_enc)
    error_imu_abs = np.abs(error_imu)
    error_tot_abs = error_enc_abs + error_imu_abs
    IAE_enc = IAE_enc + error_enc_abs
    IAE_imu = IAE_imu + error_imu_abs
    IAE_tot = IAE_imu + IAE_enc
    
    #Memperbarui error IMU dan Encoder
    error_enc_new = error_enc
    error_imu_new = error_imu
    
    #Menyimpan data plotting error dengan sampling rate 0.032s
    if current_time - last_time >= 0.032:
        t.append(current_time)
        error_imu_t.append(error_imu)
        posisi_t.append(error_enc)       
        imu_abs.append(error_imu_abs)
        enc_abs.append(error_enc_abs)
        abs_tot.append(error_tot_abs)
        jumlah.append(time_step*0.032)
        last_time = current_time
        time_step += 1

    #Keluaran robot setelah berjalan 10 detik
    if current_time-time_awal>10:    
        #Pembuatan file .csv untuk menyimpan hasil nilai error dan waktu
        filename = r"C:\Users\Nadine AK\Documents\Daily\Kuliah NOW\aSkripsi\OutputDATAs\Error_HasilOptimasiPD_GA.csv"
        now = datetime.datetime.now()
        new_filename = f"{filename}_{now.year}-{now.month}-{now.day}_{now.hour}-{now.minute}-{now.second}.csv"
        with open(new_filename, "w", newline="") as csvfile:
            fieldnames = ["Waktu(s)", "Error Sudut(deg)", "Error Posisi", "Total Error Abs"]
            writer = csv.writer(csvfile)
            writer.writerow(fieldnames)
            
            #Menuliskan data error dan waktu pada baris file .csv        
            for i in range(len(t)):
                writer.writerow([t[i], error_imu_t[i], posisi_t[i], abs_tot[i]])     
        
        #Membuat grafik plot error posisi robot terhadap waktu
        plt.figure(figsize=(12, 5))
        plt.plot(jumlah, posisi_t, label='Error posisi', alpha=0.7)
        plt.plot(jumlah, enc_abs, label='Error posisi absolut', alpha=0.4)
        window_size = 10
        for i in range(len(enc_abs)):
            window = enc_abs[max(0, i - window_size + 1): i + 1]
            window_average = sum(window) / len(window)
            enc_abs_ma.append(window_average)
        plt.plot(jumlah, enc_abs_ma, label='Moving avg', color='purple')
        
        up_posisi = np.argmax(posisi_t)
        down_posisi = np.argmin(posisi_t)
        up_posisi_abs = np.argmax(enc_abs_ma)
        
        plt.xlabel('Waktu (s)')
        plt.ylabel('Error Posisi (rad)')
        plt.title('Error Posisi Terhadap Waktu')
        plt.grid(True, which='major', linewidth=0.5, linestyle='-', color='gray', alpha=0.4)
        plt.axhline(0.0, color='black', linestyle='-',linewidth=0.7, alpha=0.8)
        
        plt.axhline(y=posisi_t[up_posisi], color='c', linestyle='--',linewidth=0.7)
        plt.text(jumlah[up_posisi], posisi_t[up_posisi], '{:.4f} rad'.format(posisi_t[up_posisi]), fontsize=8, color='black', bbox=dict(facecolor='white', alpha=0.3, edgecolor='purple'))
        
        plt.axhline(y=posisi_t[down_posisi], color='r', linestyle='--',linewidth=0.7)
        plt.text(jumlah[down_posisi], posisi_t[down_posisi], '{:.4f} rad'.format(posisi_t[down_posisi]), fontsize=8, color='black', bbox=dict(facecolor='white', alpha=0.3, edgecolor='purple'))
        
        plt.axhline(y=enc_abs_ma[up_posisi_abs], color='g', linestyle='--',linewidth=0.7)
        plt.text(jumlah[up_posisi_abs], enc_abs_ma[up_posisi_abs], '{:.4f} rad'.format(enc_abs_ma[up_posisi_abs]), fontsize=8, bbox=dict(facecolor='white', alpha=0.3, edgecolor='purple'))
        
        plt.legend(bbox_to_anchor=(1, 0.5), loc='center left')
        plt.subplots_adjust(right=0.85, left=0.085)
        plt.show()
        
        #Membuat grafik plot error sudut terhadap waktu
        plt.figure(figsize=(12, 5))
        plt.plot(jumlah, error_imu_t, label='Error sudut', alpha=0.7)
        plt.plot(jumlah, imu_abs, label='Error sudut absolut', alpha=0.4)
        window_size = 10
        for i in range(len(imu_abs)):
            window = imu_abs[max(0, i - window_size + 1): i + 1]
            window_average = sum(window) / len(window)
            imu_abs_ma.append(window_average)
        plt.plot(jumlah, imu_abs_ma, label='Moving avg', color='purple')
        
        up_angle = np.argmax(error_imu_t)
        down_angle = np.argmin(error_imu_t)
        up_angle_abs = np.argmax(imu_abs_ma)
        
        plt.xlabel('Waktu (s)')
        plt.ylabel('Error Sudut (deg)')
        plt.title('Error Sudut Terhadap Waktu')
        plt.grid(True, which='major', linewidth=0.5, linestyle='-', color='gray', alpha=0.4)
        plt.axhline(0.0, color='black', linestyle='-',linewidth=0.7, alpha=0.8)
        
        plt.axhline(y=error_imu_t[up_angle], color='c', linestyle='--',linewidth=0.7)
        plt.text(jumlah[up_angle], error_imu_t[up_angle], '{:.4f} derajat'.format(error_imu_t[up_angle]), fontsize=8, bbox=dict(facecolor='white', alpha=0.3, edgecolor='purple'))
        
        plt.axhline(y=error_imu_t[down_angle], color='r', linestyle='--',linewidth=0.7)
        plt.text(jumlah[down_angle], error_imu_t[down_angle], '{:.4f} derajat'.format(error_imu_t[down_angle]), fontsize=8, bbox=dict(facecolor='white', alpha=0.3, edgecolor='purple'))
        
        plt.axhline(y=imu_abs_ma[up_angle_abs], color='g', linestyle='--',linewidth=0.7)
        plt.text(jumlah[up_angle_abs], imu_abs_ma[up_angle_abs], '{:.4f} derajat'.format(imu_abs_ma[up_angle_abs]), fontsize=8, bbox=dict(facecolor='white', alpha=0.3, edgecolor='purple'))
        
        plt.legend(bbox_to_anchor=(1, 0.5), loc='center left')
        plt.subplots_adjust(right=0.85, left=0.085)
        plt.show()
        
        #Membuat grafik plot total error absolut terhadap waktu
        plt.figure(figsize=(12, 5))
        plt.plot(jumlah, abs_tot, label='Error Total')

        up_tot = np.argmax(abs_tot)
        
        plt.xlabel('Waktu (s)')
        plt.ylabel('Error Total (IMU+Encoder)')
        plt.title('Error Total Terhadap Waktu')
        plt.grid(True, which='major', linewidth=0.5, linestyle='-', color='gray', alpha=0.4)
        plt.axhline(0.0, color='black', linestyle='-',linewidth=0.7, alpha=0.8)
        
        plt.axhline(y=abs_tot[up_tot], color='g', linestyle='--',linewidth=0.7)
        plt.text(jumlah[up_tot], abs_tot[up_tot], '{:.4f}'.format(abs_tot[up_tot]), fontsize=8, bbox=dict(facecolor='white', alpha=0.3, edgecolor='purple'))

        plt.legend(bbox_to_anchor=(1, 0.5), loc='center left')
        plt.subplots_adjust(right=0.85, left=0.085)
        plt.show()
        
        print(f"Nilai IAE dari parameter KP: {KP} & KD: {KD} adalah {IAE_tot}")
        break