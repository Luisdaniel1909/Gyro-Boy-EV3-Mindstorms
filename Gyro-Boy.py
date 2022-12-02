#!/usr/bin/env pybricks-micropython

from ucollections import namedtuple
import urandom

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, ImageFile, SoundFile
from pybricks.tools import wait, StopWatch

# Este comando inicia el ladrillo EV3.
ev3 = EV3Brick()

# Inicializar los motores conectados a las ruedas motrices.
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

# Inicializar el motor conectado a los brazos.
arm_motor = Motor(Port.C)

# Inicializar el sensor de color. Se utiliza para detectar los colores que ordena el camino que debe seguir el robot.
color_sensor = ColorSensor(Port.S1)

# Inicializar el sensor giroscópico. Se utiliza para proporcionar información para equilibrar el robot y que este no se caiga.
gyro_sensor = GyroSensor(Port.S2)

# Inicializar el sensor ultrasónico. Se utiliza para detectar cuando el robot se acerca demasiado cerca de un obstáculo.
ultrasonic_sensor = UltrasonicSensor(Port.S4)

# Inicializar los temporizadores.
fall_timer = StopWatch()
single_loop_timer = StopWatch()
control_loop_timer = StopWatch()
action_timer = StopWatch()


# Los siguientes elementos (palabras en MAYÚSCULAS) son constantes que controlan cómo se comporta el programa.
GYRO_CALIBRATION_LOOP_COUNT = 200
GYRO_OFFSET_FACTOR = 0.0005
TARGET_LOOP_PERIOD = 15  # ms
ARM_MOTOR_SPEED = 600  # deg/s

# Las acciones se utilizarán para cambiar la forma en que conduce el robot.
Action = namedtuple('Action ', ['drive_speed', 'steering'])

# Estas son las acciones predefinidas
STOP = Action(drive_speed=0, steering=0)
FORWARD_FAST = Action(drive_speed=150, steering=0)
FORWARD_SLOW = Action(drive_speed=40, steering=0)
BACKWARD_FAST = Action(drive_speed=-75, steering=0)
BACKWARD_SLOW = Action(drive_speed=-10, steering=0)
TURN_RIGHT = Action(drive_speed=0, steering=70)
TURN_LEFT = Action(drive_speed=0, steering=-70)

# Los colores que el sensor de color puede detectar se asignan a acciones que el robot puede realizar.
ACTION_MAP = {
    Color.RED: STOP,
    Color.GREEN: FORWARD_FAST,
    Color.BLUE: TURN_RIGHT,
    Color.YELLOW: TURN_LEFT,
    Color.WHITE: BACKWARD_FAST,
}


# Esta siguiente función monitorea el sensor de color y el sensor ultrasónico.

# Es importante que no se realicen llamadas de bloqueo en esta función, de lo contrario afectará el tiempo de ciclo de 
#control en el programa principal. En cambio, cedemos al bucle de control mientras esperamos que suceda algo como esto:
#
# while not nuestracondicion:
#         yield
#
# También usamos el rendimiento para actualizar la velocidad de conducción y los valores de dirección en la pantalla principal.
# control loop:
#
#     yield action
#
def update_action():
    arm_motor.reset_angle(0)
    action_timer.reset()

    # Conduzca hacia adelante durante 4 segundos para dejar el soporte, luego deténgase.
    yield FORWARD_SLOW
    while action_timer.time() < 4000:
        yield

    action = STOP
    yield action

# Comience a verificar los sensores en los brazos. Cuando se detectan condiciones específicas, se realizarán diferentes acciones.
    while True:
        # Primero, revisamos el sensor de color. El color detectado se busca en el mapa de acción.
        new_action = ACTION_MAP.get(color_sensor.color())

# Si se encontró el color, emitir un pitido durante 0,1 segundos y luego cambiar la acción según el color detectado.
        if new_action is not None:
            action_timer.reset()
            ev3.speaker.beep(1000, -1)
            while action_timer.time() < 100:
                yield
            ev3.speaker.beep(0, -1)

           # Si la nueva acción involucra la dirección, combine la nueva dirección con la velocidad de conducción anterior. 
           # De lo contrario, utilice toda la nueva acción.
            if new_action.steering != 0:
                action = Action(drive_speed=action.drive_speed,
                                steering=new_action.steering)
            else:
                action = new_action
            yield action

        # Si la distancia medida del sensor ultrasónico es inferior a 250 milímetros, retroceda lentamente.
        if ultrasonic_sensor.distance() < 250: 
           # Retroceda lentamente mientras mueve los brazos de un lado a otro.
            yield BACKWARD_SLOW

            arm_motor.run_angle(ARM_MOTOR_SPEED, 30, wait=False)
            while not arm_motor.control.done():
                yield
            arm_motor.run_angle(ARM_MOTOR_SPEED, -60, wait=False)
            while not arm_motor.control.done():
                yield
            arm_motor.run_angle(ARM_MOTOR_SPEED, 30, wait=False)
            while not arm_motor.control.done():
                yield

            # Gire aleatoriamente a la izquierda o a la derecha durante 4 segundos mientras sigue retrocediendo lentamente.
            turn = urandom.choice([TURN_LEFT, TURN_RIGHT])
            yield Action(drive_speed=BACKWARD_SLOW.drive_speed,
                         steering=turn.steering)
            action_timer.reset()
            while action_timer.time() < 4000:
                yield

           # Sonido de Beep y luego restablezca la acción anterior antes de que el sensor ultrasónico detecte una obstrucción.
            action_timer.reset()
            ev3.speaker.beep(1000, -1)
            while action_timer.time() < 100:
                yield
            ev3.speaker.beep(0, -1)

            yield action

       # Esto agrega un pequeño retraso ya que no necesitamos leer estos sensores continuamente. 
       # Leer una vez cada 100 milisegundos es lo suficientemente rápido para evitar sobrecarga.
        action_timer.reset()
        while action_timer.time() < 100:
            yield


# Si el robot se cae en medio de una acción, los motores de los brazos podrían estar moviéndose 
# o el altavoz podría estar emitiendo un pitido, por lo que debemos detener ambos.
def stop_action():
    ev3.speaker.beep(0, -1)
    arm_motor.run_target(ARM_MOTOR_SPEED, 0)


while True:
    # Los ojos durmientes y la luz apagada nos permiten saber que el robot 
    # está esperando que se detenga cualquier movimiento antes de que el programa pueda continuar (caracteristica estetica).
    ev3.screen.load_image(ImageFile.SLEEPING)
    ev3.light.off()

    # Restablecer los sensores y variables a cero.
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    fall_timer.reset()

    motor_position_sum = 0
    wheel_angle = 0
    motor_position_change = [0, 0, 0, 0]
    drive_speed, steering = 0, 0
    control_loop_count = 0
    robot_body_angle = -0.25

    # Dado que update_action() es un generador (usa "yield" en lugar de
    # "return") en realidad no ejecuta  update_action() en este momento, sino que lo prepara para su uso posterior.
    action_task = update_action()

# Calibre la compensación del giroscopio. Esto asegura que el robot esté perfectamente inmóvil asegurándose de 
# que la velocidad medida no fluctúe más de 2 grados/s. La desviación del giroscopio puede hacer que la tasa sea 
# distinta de cero incluso cuando el robot no se está moviendo, por lo que guardamos ese valor para usarlo más adelante.
    while True:
        gyro_minimum_rate, gyro_maximum_rate = 440, -440
        gyro_sum = 0
        for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
            gyro_sensor_value = gyro_sensor.speed()
            gyro_sum += gyro_sensor_value
            if gyro_sensor_value > gyro_maximum_rate:
                gyro_maximum_rate = gyro_sensor_value
            if gyro_sensor_value < gyro_minimum_rate:
                gyro_minimum_rate = gyro_sensor_value
            wait(5)
        if gyro_maximum_rate - gyro_minimum_rate < 2:
            break
    gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT

   # Los ojos despiertos y la luz verde nos hacen saber que el robot está listo para funcionar (caracteristica estetica)
    ev3.speaker.play_file(SoundFile.SPEED_UP)
    ev3.screen.load_image(ImageFile.AWAKE)
    ev3.light.on(Color.GREEN)

    # Bucle de control principal para equilibrar el robot.
    while True:
# Este temporizador mide cuánto tiempo tarda un solo bucle. Esto se usará para ayudar a mantener el tiempo de bucle constante, 
# incluso cuando se estén realizando diferentes acciones.
        single_loop_timer.reset()

        # Esto calcula el periodo medio del bucle de control. Esto se utiliza en el cálculo de la retroalimentación de control
        # en lugar del tiempo de bucle único para filtrar las fluctuaciones aleatorias.
        if control_loop_count == 0:
            # La primera vez que se pasa por el bucle, tenemos que asignar un valor para evitar dividir por cero después.
            average_control_loop_period = TARGET_LOOP_PERIOD / 1000
            control_loop_timer.reset()
        else:
            average_control_loop_period = (control_loop_timer.time() / 1000 /
                                           control_loop_count)
        control_loop_count += 1

        # calcular el ángulo del cuerpo del robot y la velocidad
        gyro_sensor_value = gyro_sensor.speed()
        gyro_offset *= (1 - GYRO_OFFSET_FACTOR)
        gyro_offset += GYRO_OFFSET_FACTOR * gyro_sensor_value
        robot_body_rate = gyro_sensor_value - gyro_offset
        robot_body_angle += robot_body_rate * average_control_loop_period

        # calcular el ángulo de la rueda y la velocidad
        left_motor_angle = left_motor.angle()
        right_motor_angle = right_motor.angle()
        previous_motor_sum = motor_position_sum
        motor_position_sum = left_motor_angle + right_motor_angle
        change = motor_position_sum - previous_motor_sum
        motor_position_change.insert(0, change)
        del motor_position_change[-1]
        wheel_angle += change - drive_speed * average_control_loop_period
        wheel_rate = sum(motor_position_change) / 4 / average_control_loop_period

        # Este es el principal cálculo de retroalimentación de control.
        output_power = (-0.01 * drive_speed) + (0.8 * robot_body_rate +
                                                15 * robot_body_angle +
                                                0.08 * wheel_rate +
                                                0.12 * wheel_angle)
        if output_power > 100:
            output_power = 100
        if output_power < -100:
            output_power = -100

        # Acciona los motores.
        left_motor.dc(output_power - 0.1 * steering)
        right_motor.dc(output_power + 0.1 * steering)

        # Comprueba si el robot se ha caído. Si la velocidad de salida es +/-100% 
        # durante más más de un segundo, sabemos que ya no estamos equilibrados correctamente.
        if abs(output_power) < 100:
            fall_timer.reset()
        elif fall_timer.time() > 1000:
            break

        # Esto ejecuta update_action() hasta la siguiente sentencia "yield".
        action = next(action_task)
        if action is not None:
            drive_speed, steering = action

        # Asegúrese de que el tiempo de bucle es al menos 
        # TARGET_LOOP_PERIOD. El cálculo de la potencia de salida anterior depende de tener una cierta cantidad de tiempo en cada bucle.
        wait(TARGET_LOOP_PERIOD - single_loop_timer.time())

    # Manejar la caída. Si llegamos a este punto del programa, significa que el robot se ha caído.

    # Parar todos los motores.
    stop_action()
    left_motor.stop()
    right_motor.stop()

    # Los ojos desorbitados y la luz roja nos hacen saber que el robot perdió el equilibrio (carateristica estetica).
    ev3.light.on(Color.RED)
    ev3.screen.load_image(ImageFile.KNOCKED_OUT)
    ev3.speaker.play_file(SoundFile.SPEED_DOWN)

    # Espera unos segundos antes de intentar equilibrar de nuevo.
    wait(3000)
