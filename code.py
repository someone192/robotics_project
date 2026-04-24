import random
import math
import time
import serial

# Параметры грузовика
LENGTH = 5  # длина (x), 5 ячеек
WIDTH = 4  # ширина (y), 4 ячейки
TARGET_CENTER = (LENGTH / 2, WIDTH / 2)  # целевой центр тяжести: (2.5, 2)

# Среднее время погрузки 1 коробки в секундах
average_time = 33.75

# Позиции коробок по номерам
boxes_positions = {
    1: (4.5,0.5),
    2: (4.5,1.5),
    3: (4.5,2.5),
    4: (4.5,3.5),
    5: (4.5,4.5),
    6: (5.5,0.5),
    7: (5.5,1.5),
    8: (5.5,2.5),
    9: (5.5,3.5),
    10: (5.5,4.5),
    11: (6.5,0.5),
    12: (6.5,1.5),
    13: (6.5,2.5),
    14: (6.5,3.5),
    15: (6.5,4.5),
    16: (7.5,0.5),
    17: (7.5,1.5),
    18: (7.5,2.5),
    19: (7.5,3.5),
    20: (7.5,4.5)
}

#def write_read(x):
 #   arduino.write(bytes(x,   'utf-8'))
 #   time.sleep(1.05)
 #   data = arduino.readline()
 #   return   data

# Функция отправки команд на arduino
def send_command(command):
    arduino.write((command + '\n').encode())  # Отправляем команду
    time.sleep(0.1)  # Ждём немного
    while arduino.in_waiting > 0:  # Читаем ответ
        response = arduino.readline().decode().strip()
        print(f"Arduino: {response}")   # Выводим ответ arduino

# Функция для вычисления центра тяжести
def calculate_center_of_mass(positions, masses):
    total_mass = sum(masses)
    x_c = sum(m * x for (x, y), m in zip(positions, masses)) / total_mass
    y_c = sum(m * y for (x, y), m in zip(positions, masses)) / total_mass
    return x_c, y_c


# Функция ошибки (квадрат расстояния от центра тяжести до целевого центра)
def calculate_error(positions, masses):
    x_c, y_c = calculate_center_of_mass(positions, masses)
    return (x_c - TARGET_CENTER[0]) ** 2 + (y_c - TARGET_CENTER[1]) ** 2


# Жадное начальное размещение: тяжелые ближе к центру
def greedy_initial_placement(masses):
    # Координаты всех ячеек
    grid = [(i + 0.5, j + 0.5) for i in range(LENGTH) for j in range(WIDTH)]
    # Сортируем ячейки по расстоянию до центра
    grid.sort(key=lambda pos: (pos[0] - TARGET_CENTER[0]) ** 2 + (pos[1] - TARGET_CENTER[1]) ** 2)
    # Сортируем массы по убыванию
    sorted_masses = sorted(masses, reverse=True)
    # Назначаем каждой массе позицию
    positions = grid[:len(masses)]
    return positions, sorted_masses


# Метод имитации отжига
def simulated_annealing(masses, time_limit=300):  # time_limit в секундах (5 минут = 300 сек)
    # Начальное размещение
    positions, current_masses = greedy_initial_placement(masses)
    current_error = calculate_error(positions, current_masses)
    best_positions, best_error = positions[:], current_error

    # Параметры отжига
    T = 1000  # начальная температура
    cooling_rate = 0.995
    min_T = 0.01
    start_time = time.time()

    while T > min_T and (time.time() - start_time) < time_limit:
        # Создаем новое решение: меняем местами две случайные коробки
        new_positions = positions[:]
        i, j = random.sample(range(len(masses)), 2)
        new_positions[i], new_positions[j] = new_positions[j], new_positions[i]

        # Вычисляем новую ошибку
        new_error = calculate_error(new_positions, current_masses)

        # Принимаем новое решение
        if new_error < current_error or random.random() < math.exp(-(new_error - current_error) / T):
            positions = new_positions
            current_error = new_error

        # Обновляем лучшее решение
        if current_error < best_error:
            best_positions = positions[:]
            best_error = current_error

        # Охлаждаем температуру
        T *= cooling_rate

    return best_positions


# Использование
if __name__ == "__main__":
    # Пример: 20 коробок с массами от 1 до 20
    masses = [20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1]

    # Запускаем алгоритм
    start_time = time.time()
    optimal_positions = simulated_annealing(masses)
    end_time = time.time()

    arduino = serial.Serial(port='COM18', baudrate=115200, timeout=.1)
    time.sleep(2)

    # Ожидание сигнала Arduino о готовности
    response = ""
    while response != "CoreXY System Ready":
        time.sleep(2)  # Ждём выполнения
        while arduino.in_waiting > 0:  # Читаем ответ
            response = arduino.readline().decode().strip()
    print(response)
    send_command(f"G28")
    response = ""
    while response != "OK":
        time.sleep(2)  # Ждём выполнения
        while arduino.in_waiting > 0:  # Читаем ответ
            response = arduino.readline().decode().strip()
    print(response)


    # Проверка центра тяжести
    x_c, y_c = calculate_center_of_mass(optimal_positions, masses)
    print(f"Центр тяжести: ({x_c:.2f}, {y_c:.2f})")
    print(f"Целевой центр: {TARGET_CENTER}")
    error = calculate_error(optimal_positions, masses)
    print(f"Ошибка: {error:.4f}")

    # Вывод результатов для передачи и исполнения
    print(f"Время выполнения программы: {end_time - start_time:.2f} секунд")
    print(f"Расчетное время погрузки:{average_time * len(masses) // 60} минут {average_time * len(masses) % 60} секунд")
    print(f"Оптимальное расположение коробок (координаты):")
    start_time_cargo = time.time()

    for i, (pos, mass) in enumerate(zip(optimal_positions, masses)):
        print(f"Коробка {i + 1}: масса = {mass}, позиция = {pos}")
        send_command(f"G0 X{boxes_positions[i + 1][1] * 68.75} Y{boxes_positions[i + 1][0] * 68.75}")
        print(f"G0 X{boxes_positions[i + 1][1] * 68.75} Y{boxes_positions[i + 1][0] * 68.75}")
        response = ""
        while response != "OK":
            time.sleep(2)  # Ждём выполнения
            while arduino.in_waiting > 0:  # Читаем ответ
                response = arduino.readline().decode().strip()
        print(response)
        send_command(f"G0 X{pos[0] * 68.75} Y{pos[1] * 68.75}")
        print(f"G0 X{pos[0] * 68.75} Y{pos[1] * 68.75}")
        response = ""
        while response != "OK":
            time.sleep(2)  # Ждём выполнения
            while arduino.in_waiting > 0:  # Читаем ответ
                response = arduino.readline().decode().strip()
        print(response)
        time.sleep(1)
        #value = write_read(f"{boxes_positions[i + 1]}, {(pos[0]* 20,pos[1] * 20)}")
        #print(value)
    end_time_cargo = time.time()
    print(f"Время выполнения погрузки: {end_time_cargo - start_time_cargo} секунд")
    # Контрольная проверка центра тяжести
    x_c, y_c = calculate_center_of_mass(optimal_positions, masses)
    print(f"Центр тяжести: ({x_c:.2f}, {y_c:.2f})")
    print(f"Целевой центр: {TARGET_CENTER}")
    error = calculate_error(optimal_positions, masses)
    print(f"Ошибка: {error:.4f}")
