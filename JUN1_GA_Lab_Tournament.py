from controller import Supervisor
import deap
import csv
import numpy as np
import random
import matplotlib.pyplot as plt
from deap import base, creator, tools, algorithms
import datetime

robot = Supervisor()
#TIMESTEP 32
timestep = int(robot.getBasicTimeStep())

#Titik posisi dan rotasi awal robot pada world
sbr_node = robot.getFromDef('SBR')
translation_field = sbr_node.getField('translation')
rotation_field = sbr_node.getField("rotation")
trans_field_awal = [0.0, 0.0, 0.0]
trans_field_awal = translation_field.getSFVec3f()
rotation_field_awal = rotation_field.getSFRotation()

#Variabel robot jatuh
jatuh_values = [0.0, 0.0, 0.0, 0.0]
 
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
position_values[0] = left_encoder.getValue() - 5.1821673902967476
position_values[1] = right_encoder.getValue()
last_position_values = [0.0,0.0]
lala_posi_val = [0.0,0.0]

#Variabel error
error_enc_new = 0.0
error_enc = 0.0
error_imu_new = 0.0
error_imu = 0.0
i = 0

#Variabel IAE
IAE_enc = 0.0
IAE_imu = 0.0
IAE_tot = 0.0
error_imu_IAE = 0.0
error_enc_IAE = 0.0

#Variabel PD ENCODER
Kp_enc = 0.06
Kd_enc = 21

#Variabel PD IMU
KP_RANGE = (100.0, 200.0) #Trial & Error: 175
KP_STEP = 1.0
KD_RANGE = (0.1, 5.1) #Trial & Error: 0.3
KD_STEP = 0.1

#Batasan untuk individu saat proses crossover dan mutasi
MIN = [KP_RANGE[0], KD_RANGE[0]]
MAX = [KP_RANGE[1], KD_RANGE[1]]

#Variabel penentuan 1 pasangan KP KD tiap individu
N_CYCLES = 1

#Variabel penyimpanan KP KD dan fitness
KP_Store = []
KD_Store = []
FIT_Store = []

#Fungsi nilai acak dari bounding pada KP KD dengan step
def random_uniform_with_step(low, high, step):
    value = low + step * random.randint(0, int((high - low) / step))
    return round(value, 1)

#-----------------------------SIMULASI BERJALAN----------------------------
while robot.step(timestep) != -1:
    initial_pitch_angle = inemu.getRollPitchYaw()[1]

    #Membuat kelas fitness dan individual
    creator.create("FitnessMax", base.Fitness, weights=(1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMax)
    
    #Membuat toolbox lib DEAP
    toolbox = base.Toolbox()
    
    #Meregister individu KP KD dengan nilai acak dan step
    toolbox.register("individual_KP", random_uniform_with_step, *KP_RANGE, KP_STEP)
    toolbox.register("individual_KD", random_uniform_with_step, *KD_RANGE, KD_STEP)
    toolbox.register("individual", tools.initCycle, creator.Individual, (toolbox.individual_KP, toolbox.individual_KD), n=N_CYCLES)

    #Fungsi membuat populasi awal dengan 20 individu beragam
    def create_population(toolbox, n):
        population = []
        while len(population) < n:
            individual = toolbox.individual()
            while individual in population:
                individual = toolbox.individual()
            population.append(individual)
        return population
    toolbox.register("population", create_population, toolbox)
    pop = toolbox.population(n=20)
    
    #Fungsi perhitungan fitness dari individu
    def fitness(individual):
        global position_values, initial_pitch_angle, jatuh
        global Kp_enc, Kd_enc
        global error_imu, error_enc, error_imu_new, error_enc_new
        global IAE_imu, IAE_enc, IAE_tot, error_imu_IAE, error_enc_IAE
        global KP_Store, KD_Store, FIT_Store
        start = robot.getTime()
        
        #Deklarasi KP KD sebagai individu
        KP, KD = individual
        print ("KP: {} __ KD: {}".format(KP, KD))
        KP_Store.append(KP)
        KD_Store.append(KD)
        
        #Training GA robot berjalan selama 10 detik
        while robot.step(timestep) != -1:
            #Membaca nilai rotasi robot melalui supervisor world
            jatuh_values = rotation_field.getSFRotation()
            jatuh = np.abs(jatuh_values[3])
            
            #PD Encoder
            last_position_values[0] = left_encoder.getValue() - lala_posi_val[0]
            error_enc = position_values[0] - last_position_values[0]          
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
            
            #Menghitung IAE
            error_enc_IAE = np.abs(error_enc)
            error_imu_IAE = np.abs(error_imu)
            IAE_enc = IAE_enc + error_enc_IAE
            IAE_imu = IAE_imu + error_imu_IAE
            IAE_tot = IAE_imu + IAE_enc
            
            #Robot berhenti apabila terjatuh dan diberi penalti
            if jatuh > 1.5 :
                speed = 0.0
                left_motor.setVelocity(speed)
                right_motor.setVelocity(speed)
                IAE_tot += 10000
            
            #Memperbarui error IMU dan Encoder
            error_enc_new = error_enc
            error_imu_new = error_imu
            
            #Keluaran robot setelah berjalan 10 detik
            if robot.getTime()-start>10:
                #Perhitungan fitness
                fitness_gene = 1 / ((np.abs(IAE_tot)) + 1)
                print(f"Fit: {fitness_gene} ")
                print("...")
                FIT_Store.append(fitness_gene)
                
                #Robot diatur kembali ke posisi semula dengan nilai awal
                translation_field.setSFVec3f(trans_field_awal)
                rotation_field.setSFRotation(rotation_field_awal)
                sbr_node.resetPhysics()
                lala_posi_val[0] = last_position_values[0] + lala_posi_val[0]
                error_enc_new = 0.0
                error_imu_new = 0.0
                IAE_imu = 0
                IAE_enc = 0
                IAE_tot = 0
                break

        return fitness_gene
        
    #Evaluasi fitness
    toolbox.register("evaluate", fitness)

    #Fungsi Tournament Selection
    toolbox.register("select", tools.selTournament, tournsize=5)
    
    #Fungsi pengecekan batasan individu
    def checkBounds(min, max):
        def decorator(func):
            def wrapper(*args, **kargs):
                offspring = func(*args, **kargs)
                for child in offspring:
                    for i in range(len(child)):
                        if child[i] > max[i]:
                            child[i] = max[i]
                        elif child[i] < min[i]:
                            child[i] = min[i]
                return offspring
            return wrapper
        return decorator
    
    #Fungsi Blend Crossover
    def blend_crossover(ind1: list, ind2: list) -> tuple:
        alpha = 0.5
        for i in range(len(ind1)):
            if random.random() < 0.3:
                ind1[i] = ind1[i] + alpha * (ind2[i] - ind1[i])
                ind2[i] = ind2[i] + alpha * (ind1[i] - ind2[i])
            ind1[i] = round(ind1[i], 1)
            ind2[i] = round(ind2[i], 1)
        return ind1, ind2
    toolbox.register("mate", blend_crossover)
    toolbox.decorate("mate", checkBounds(MIN, MAX))
    
    #Fungsi Gaussian Mutation 
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=2, indpb=0.3)
    toolbox.decorate("mutate", checkBounds(MIN, MAX))
  
    #Inisiasi variabel untuk daftar penyimpanan nilai fitness dan individu
    fitness_values = []

    #Inisiasi variabel untuk menyimpan individu dengan fitness tertinggi dan terendah 
    best_individual_overall = None
    best_fitness_overall = float("-inf")
    best_generation = 0
    worst_individual_overall = None
    worst_fitness_overall = float("-inf")
    worst_generation = 0
    
    #Pembuatan file .csv untuk menyimpan hasil nilai KP KD dan Fitness
    filename = r"/home/trkb/Documents/TWIPR/Output/Lab_100Gen.csv"
    now = datetime.datetime.now()
    new_filename = f"{filename}_{now.year}-{now.month}-{now.day}_{now.hour}-{now.minute}-{now.second}.csv"
    with open(new_filename, "w", newline="") as csvfile:
        fieldnames = ["Generasi", "KP", "KD", "Fitness"]
        writer = csv.writer(csvfile)
        writer.writerow(fieldnames)
   
        #Jumlah generasi yang digunakan dalam algoritma genetika
        NGEN = 120
        
        #Inisiasi variabel untuk menyimpan 5 generasi terbaik dan 1 generasi terburuk
        top_5_gen_fitness = []
        bad_1_gen_fitness = []
        
        #Algoritma genetika berjalan untuk tiap generasi
        for gen in range(NGEN):
            #Pemilihan keturunan generasi selanjutnya dengan fungsi lib DEAP
            offspring = algorithms.varAnd(pop, toolbox, cxpb=0.8, mutpb=0.4)
            
            #Evaluasi fitness dari tiap individu pada populasi
            fits = toolbox.map(toolbox.evaluate, offspring)
            
            #Menyimpan nilai fitness sesuai dengan individu yang dievaluasi
            for fit, ind in zip(fits, offspring):
                ind.fitness.values = (fit,) 

            #Membuat populasi keturunan untuk generasi selanjutnya dengan ukuran 20
            new_pop = toolbox.select(offspring, k=20)
            pop = new_pop
            
            #Pemilihan individu dengan fitness terbaik dan terburuk dalam populasi
            best_individual = tools.selBest(new_pop, k=1)[0]
            best_fit = best_individual.fitness.values[0]
            worst_individual = tools.selBest(new_pop, k=1)[0]
            
            #Mencetak hasil generasi dengan fitness dan individu terbaik
            print(f"Generation {gen+1}:")
            print(f"  Best fitness: {best_fit}")
            print(f"  Best individual: {best_individual}")
            print("-------------------------------------")

            #Menyimpan nilai fitness terbaik dari tiap generasi
            fitness_values.append(best_fit)
        
            #Memperbarui nilai fitness dan individual terbaik
            if best_fit > best_fitness_overall:
                best_individual_overall = best_individual
                best_fitness_overall = best_fit
                best_generation = gen + 1

            #Menambahkan nilai fitness dan generasi pada daftar 5 generasi dengan fitness tertinggi
            top_5_gen_fitness.append((gen, best_fit, best_individual[0], best_individual[1]))
            top_5_gen_fitness = sorted(top_5_gen_fitness, key=lambda x: x[1], reverse=True)[:5]
            
            #Menambahkan nilai fitness dan generasi pada daftar generasi dengan fitness terendah
            bad_1_gen_fitness.append((gen, best_fit, worst_individual[0], worst_individual[1]))
            bad_1_gen_fitness = sorted(bad_1_gen_fitness, key=lambda x: x[1], reverse=False)[:1]
        
        #Menuliskan data KP KD dan Fitness pada baris file .csv
        gencsv = 1        
        for i in range(len(KP_Store)):
            writer.writerow([gencsv, KP_Store[i], KD_Store[i], FIT_Store[i]]) 
            if (i + 1) % 20 == 0:
                gencsv += 1
        break
        
#Mencetak hasil optimasi parameter PD menggunakan algoritma genetika
print(f"Generasi Terbaik: {best_generation}")
print(f"Individu: {best_individual_overall}")
print(f"Fitness: {best_fitness_overall}")
print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
print("TOP 5 Generasi dengan nilai fitness tertinggi")
for i in range(5):
    if i < len(top_5_gen_fitness):
        gen, fit, KP, KD = top_5_gen_fitness[i]
        print(f"Peringkat {i+1} = Generasi {gen+1} dengan fitness: {fit} (KP: {KP}  &  KD: {KD})")
    else:
        break
print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
print("Generasi terburuk dengan fitness paling rendah")
for i in range(1):
    if i < len(bad_1_gen_fitness):
        gen, fit, KP, KD = bad_1_gen_fitness[i]
        print(f"Generasi {gen+1} dengan fitness: {fit} (KP: {KP}  &  KD: {KD})")
    else:
        break

#Membuat grafik plot fitness per individu
plt.figure(figsize=(30, 5))
plt.subplot(1, 1, 1)
plt.plot(range(1, len(FIT_Store)+1), FIT_Store)
peak_point = np.argmax(FIT_Store) + 1
lowest_point = np.argmin(FIT_Store) + 1
plt.xlabel("Individu")
plt.ylabel("Fitness")
plt.title("Fitness per Individu")
plt.grid(True, which='major', linewidth=0.5, linestyle='-', color='gray')
plt.tick_params(axis='both', labelsize=8)
plt.tight_layout()
plt.axvline(peak_point, color='r', linestyle='--',linewidth=0.7)
plt.axvline(lowest_point, color='g', linestyle='--', linewidth=0.7)
plt.axhline(y=FIT_Store[peak_point-1], color='r', linestyle='--',linewidth=0.7)
plt.axhline(y=FIT_Store[lowest_point-1], color='g', linestyle='--',linewidth=0.7)
plt.text(peak_point, FIT_Store[peak_point-1], 'Peak: Individu ' + str(peak_point), fontsize=8, color='r')
plt.text(lowest_point, FIT_Store[lowest_point-1], 'Lowest: Individu ' + str(lowest_point), fontsize=8, color='g')
plt.xticks(range(1, len(FIT_Store)+1, 20), [str(i) for i in range(1, len(FIT_Store)//20 + 1)])
plt.show()

#Membuat grafik plot fitness per generasi
plt.figure(figsize=(15, 5))
plt.subplot(1, 1, 1)
plt.plot(range(1, NGEN+1), fitness_values)
peak_point = np.argmax(fitness_values) + 1
lowest_point = np.argmin(fitness_values) + 1
plt.xlabel("Generasi")
plt.ylabel("Fitness")
plt.title("Fitness per Generasi")
plt.grid(True, which='major', linewidth=0.5, linestyle='-', color='gray')
plt.tick_params(axis='both', labelsize=8)
plt.tight_layout()
plt.axvline(peak_point, color='r', linestyle='--',linewidth=0.7)
plt.axvline(lowest_point, color='g', linestyle='--', linewidth=0.7)
plt.axhline(y=fitness_values[peak_point-1], color='r', linestyle='--',linewidth=0.7)
plt.axhline(y=fitness_values[lowest_point-1], color='g', linestyle='--',linewidth=0.7)
plt.text(peak_point, fitness_values[peak_point-1], 'Peak: Gen ' + str(peak_point), fontsize=8, color='r')
plt.text(lowest_point, fitness_values[lowest_point-1], 'Lowest: Gen ' + str(lowest_point), fontsize=8, color='g')
plt.xticks(range(10, NGEN, 10), [str(i) for i in range(10,NGEN, 10)])
plt.show()