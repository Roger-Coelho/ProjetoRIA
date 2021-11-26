# coding=utf-8
# Insert in a script in Coppelia
# simRemoteApi.start(19999)
try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

import numpy as np
import matplotlib.pyplot as plt
import time
import math
from skimage.draw import line


def readSensorData(clientId=-1,
                   range_data_signal_id="hokuyo_range_data",
                   angle_data_signal_id="hokuyo_angle_data"):
    # the first call should be non-blocking to avoid getting out-of-sync angle data
    returnCodeRanges, string_range_data = sim.simxGetStringSignal(clientId, range_data_signal_id,
                                                                  sim.simx_opmode_streaming)

    # the second call should block to avoid out-of-sync scenarios
    # between your python script and the simulator's main loop
    # (your script may be slower than the simulator's main loop, thus
    # slowing down data processing)
    returnCodeAngles, string_angle_data = sim.simxGetStringSignal(clientId, angle_data_signal_id,
                                                                  sim.simx_opmode_blocking)

    # check the if both data were obtained correctly
    if returnCodeRanges == 0 and returnCodeAngles == 0:
        # unpack data from range and sensor messages
        raw_range_data = sim.simxUnpackFloats(string_range_data)
        raw_angle_data = sim.simxUnpackFloats(string_angle_data)

        return raw_range_data, raw_angle_data

    # return none in case were nothing was gotten from the simulator
    return None


def draw_laser_data(laser_data, max_sensor_range=5):
    fig = plt.figure(figsize=(6, 6), dpi=100)
    ax = fig.add_subplot(111, aspect='equal')

    for i in range(len(laser_data)):
        ang, dist = laser_data[i]

        # Quando o feixe não acerta nada, retorna o valor máximo (definido na simulação)
        # Logo, usar um pequeno limiar do máximo para considerar a leitura
        if (max_sensor_range - dist) > 0.1:
            x = dist * np.cos(ang)
            y = dist * np.sin(ang)
            c = 'r'
            if ang < 0:
                c = 'b'
            ax.plot(x, y, 'o', color=c)

    ax.plot(0, 0, 'k>', markersize=10)
    # plt.plot(x, y)
    plt.show()

    ax.grid()
    ax.set_xlim([-max_sensor_range, max_sensor_range])
    ax.set_ylim([-max_sensor_range, max_sensor_range])


print('Program started')

#########################################################################3
# INICIALIZAÇÃO DE PARÂMETROS E DA GRID - INÍCIO

LARGGRID = 500
ALTGRID = 500
RES = 0.02
RANGE_MAX = 5
RANGE_LIMIT = 0.3
PRIORI = 0.5

fig = plt.figure(figsize=(8, 8), dpi=100)
ax = fig.add_subplot(111, aspect='equal')

map_size = np.array([500, 500])
cell_size = 1

rows, cols = (map_size / cell_size).astype(int)

m = np.random.uniform(low=0.0, high=1.0, size=(rows, cols))
# inicialização das células da grid com valor de probabilidade desconhecido
m[::, ::] = 0.5

# a primeira e última célula são inicializadas para que o valor 0.5 seja o intermediário e a matriz toda cinza
m[0, 0] = 1
m[499, 499] = 0

# INICIALIZAÇÃO DE PARÂMETROS E DA GRID - FIM
###############################################################################


sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        print('Number of objects in the scene: ', (objs))
    else:
        print('Remote API function call returned with errlenor code: ', res)

    # Iniciando a simulação
    # Deve usar a porta do 'continuous remote API server services' (remoteApiConnections.txt)
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

    print('Connected to remote API server')
    sim.simxAddStatusbarMessage(clientID, 'Iniciando...', sim.simx_opmode_oneshot_wait)
    time.sleep(0.02)

    # Handle para o Robô
    robotname = 'Pioneer_p3dx'
    erro, robotHandle = sim.simxGetObjectHandle(clientID, robotname, sim.simx_opmode_oneshot_wait)

    # Handle para as juntas das RODAS
    returnCode, l_wheel = sim.simxGetObjectHandle(clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
    returnCode, r_wheel = sim.simxGetObjectHandle(clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)

    # Criar stream de dados
    [returnCode, positionrobot] = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_streaming)
    [returnCode, orientationrobot] = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_streaming)
    time.sleep(2)

    # Handle para os dados do LASER
    laser_range_data = "hokuyo_range_data"
    laser_angle_data = "hokuyo_angle_data"

    # Geralmente a primeira leitura é inválida (atenção ao Operation Mode)
    # Em loop até garantir que as leituras serão válidas
    returnCode = 1
    while returnCode != 0:
        returnCode, range_data = sim.simxGetStringSignal(clientID, laser_range_data, sim.simx_opmode_streaming + 10)

    # Prosseguindo com as leituras
    raw_range_data, raw_angle_data = readSensorData(clientID, laser_range_data, laser_angle_data)
    laser_data = np.array([raw_angle_data, raw_range_data]).T

    # print('INFORMAÇÕES DO LASER')
    print(laser_data)
    # draw_laser_data(laser_data, 5)
    # print('QUANTIDADE DE LEITURAS: ', len(laser_data))

    returnCode, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    print('Pos: ', pos)

    # Dados do Pioneer
    L = 0.381  # Metros
    r = 0.0975  # Metros

    t = 0
    # Lembrar de habilitar o 'Real-time mode'
    startTime = time.time()
    lastTime = startTime
    dt = 0
    i = 0
    while t < 240:

        now = time.time()
        dt = now - lastTime
        # sim.simxAddStatusbarMessage(clientID, str(i) + ' - DT: ' + str(dt), sim.simx_opmode_oneshot_wait)

        # Fazendo leitura do laser
        raw_range_data, raw_angle_data = readSensorData(clientID, laser_range_data, laser_angle_data)
        laser_data = np.array([raw_angle_data, raw_range_data]).T
        # print('Range: ', raw_range_data)

        returnCode, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
        posX, posY, posZ = pos
        # print('Pos: ', pos)

        returnCode, th = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
        tx, ty, theta = th
        print('Orientation: ', theta)

        ################################################################################
        # MAPEAMENTO -INICIO
        for i in range(len(raw_range_data)):
            if raw_angle_data[i] < RANGE_MAX * RANGE_LIMIT:
                taxaOC = 0.9
            else:
                taxaOC = 0.48

            # Cálculo da posição xL, yL de onde o laser bateu
            xL = math.cos(raw_angle_data[i] + theta) * raw_range_data[i] + posX
            yL = math.sin(raw_angle_data[i] + theta) * raw_angle_data[i] + posY
            # posX e posY são as coordenadas do robô na GRID

            # Conversão da posição do robô no ambiente para a Grid
            posXGrid = (posX / RES) + (LARGGRID / 2)
            posYGrid = (posY / RES) + (ALTGRID / 2)

            # Conversão da posição de onde o laser bateu no ambiente para a Grid
            xLGrid = (xL / RES) + (LARGGRID / 2)
            yLGrid = (yL / RES) + (ALTGRID / 2)

            # Cálculo de todas as células de acordo com o algoritmo de Bresenham
            line_bresenham = np.zeros((rows, cols), dtype=np.uint8)
            xi = posXGrid
            yi = posYGrid
            xoi = xLGrid
            yoi = yLGrid
            # x é coluna; y é linha
            rr, cc = line(yi, xi, yoi, xoi)  # r0, c0, r1, c1
            line_bresenham[rr, cc] = 1

            for j in range(len(rr)):
                linha = rr[j]
                coluna = cc[j]
                m[linha, coluna] = 1 - pow(1 + (taxaOC / (1 - taxaOC)) * ((1 - PRIORI) / PRIORI) * (m[linha, coluna] / (1 - m[linha, coluna])), -1);

                if taxaOC > 0.5:
                    taxaOC = 0.48
                else:
                    taxaOC = 0.95

        # MAPEAMENTO - FIM
        ################################################################################

        # Velocidade básica (linear, angular)
        v = 0
        w = np.deg2rad(0)

        ################################################################################
        # NAVEGAÇÃO - INÍCIO
        frente = int(len(laser_data) / 2)
        lado_direito = int(len(laser_data) * 1 / 4)
        lado_esquerdo = int(len(laser_data) * 3 / 4)

        if laser_data[frente, 1] < 1:
            v = 0.1
            w = np.deg2rad(-15)
        elif laser_data[lado_direito, 1] < 1:
            v = 0.1
            w = np.deg2rad(15)
        elif laser_data[lado_esquerdo, 1] < 1:
            v = 0.1
            w = np.deg2rad(-15)
        else:
            v = 0.5
            w = 0

        sim.simxAddStatusbarMessage(clientID, str(i) + '- Frente: ' + str(laser_data[frente, 1]) + ' - Direito: ' + str(
            laser_data[lado_direito, 1]) + ' - Esquerdo: ' + str(laser_data[lado_esquerdo, 1]),
                                    sim.simx_opmode_oneshot_wait)

        # Isso é o modelo cinemático
        wl = v / r - (w * L) / (2 * r)
        wr = v / r + (w * L) / (2 * r)

        # Enviando velocidades
        sim.simxSetJointTargetVelocity(clientID, l_wheel, wl, sim.simx_opmode_streaming + 5)
        sim.simxSetJointTargetVelocity(clientID, r_wheel, wr, sim.simx_opmode_streaming + 5)

        # Parando o robô
        sim.simxSetJointTargetVelocity(clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait)

        t = t + dt
        i = i + 1
        lastTime = now

    # NAVEGAÇÃO - FIM
    ################################################################################

    # Parando o robô
    sim.simxSetJointTargetVelocity(clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait)

    #draw_laser_data(laser_data, 5)

    # Parando a simulação
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)

    plt.imshow(m, cmap='Greys', origin='upper', extent=(0, cols, rows, 0))
    plt.show(10)

else:
    print('Failed connecting to remote API server')

print('Program ended')
