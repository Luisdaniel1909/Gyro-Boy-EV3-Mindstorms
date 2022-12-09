#!/usr/bin/env pybricks-micropython

# La biblioteca urandom nos permite generar números aleatorios, o pseudoaleatorios. 
# Permite el acceso al ruido ambiental recogido de dispositivos y otras fuentes
import urandom


#importar modulos de proposito general
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.parameters import Port, Button, Color, Direction
from pybricks.media.ev3dev import Image, ImageFile, SoundFile
from pybricks.tools import wait, StopWatch


class Puppy:
    # Estas constantes se utilizan para posicionar las piernas.
    ANGULO_MEDIO_SUPERIOR = 25
    ANGULO_DE_PIE = 65
    ANGULO_DE_ESTIRAMIENTO = 125

    # Estas constantes son para posicionar la cabeza.
    ANGULO_DE_CABEZA_ARRIBA = 0
    ANGULO_CABEZA_ABAJO = 0

    # Estas constantes son para los ojos.
    OJOS_NEUTRALES = Image(ImageFile.NEUTRAL)
    OJOS_CANSADOS = Image(ImageFile.TIRED_MIDDLE)
    OJO_IZQUIERDO_CANSADO = Image(ImageFile.TIRED_LEFT)
    OJO_DERECHO_CANSADO = Image(ImageFile.TIRED_RIGHT)
    OJOS_DURMIENDO = Image(ImageFile.SLEEPING)
    OJOS_HERIDOS = Image(ImageFile.HURT)
    OJOS_ENOJADOS = Image(ImageFile.ANGRY)
    OJOS_CORAZON = Image(ImageFile.LOVE)
    OJOS_BIZCOS = Image(ImageFile.TEAR)  # la lágrima se borra después

    def __init__(self):
        # Inicializar el ladrillo EV3.
        self.ev3 = EV3Brick()

        # Inicializar los motores conectados a las patas traseras.
        self.left_leg_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
        self.right_leg_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)

        #Inicializar el motor conectado al cabezal.
        #El tornillo sin fin mueve 1 diente por rotación. 
        #Está conectado a un engranaje de 24 dientes. 
        #El engranaje de 24 dientes está conectado a engranajes paralelos 
        #de 12 dientes a través de un eje. Los engranajes de 12 dientes se conectan 
        #con engranajes de 36 dientes.
        self.head_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE,
                                gears=[[1, 24], [12, 36]])

        #Inicialice el Sensor de Color. 
        #Se utiliza para detectar los colores al alimentar al cachorro.
        self.color_sensor = ColorSensor(Port.S4)

        #Inicializa el sensor táctil. Se utiliza para detectar cuando alguien acaricia al cachorro.
        self.touch_sensor = TouchSensor(Port.S1)

        self.acaricia_count_timer = StopWatch()
        self.alimentar_count_timer = StopWatch()
        self.count_changed_timer = StopWatch()

        # Estos atributos se inicializan más tarde en el método reset().
        self.acaricia_target = None
        self.alimentar_target = None
        self.acaricia_count = None
        self.alimentar_count = None

        # Estos atributos son utilizados por las propiedades.
        self._behavior = None
        self._behavior_changed = None
        self._eyes = None
        self._eyes_changed = None

        # Estos atributos se utilizan en la actualización de los ojos
        self.eyes_timer_1 = StopWatch()
        self.eyes_timer_1_end = 0
        self.eyes_timer_2 = StopWatch()
        self.eyes_timer_2_end = 0
        self.eyes_closed = False

        # Estos atributos son utilizados por el comportamiento lúdico.
        self.playful_timer = StopWatch()
        self.playful_bark_interval = None

        # Estos atributos se utilizan en los métodos de actualización.
        self.prev_petted = None
        self.prev_color = None

    def adjust_head(self):
        """Utiliza los botones arriba y abajo del ladrillo EV3 para ajustar la 
        cabeza del cachorro hacia arriba o hacia abajo.
        """
        self.ev3.screen.load_image(ImageFile.EV3_ICON)
        self.ev3.light.on(Color.ORANGE)

        while True:
            buttons = self.ev3.buttons.pressed()
            if Button.CENTER in buttons:
                break
            elif Button.UP in buttons:
                self.head_motor.run(20)
            elif Button.DOWN in buttons:
                self.head_motor.run(-20)
            else:
                self.head_motor.stop()
            wait(100)

        self.head_motor.stop()
        self.head_motor.reset_angle(0)
        self.ev3.light.on(Color.GREEN)

    def move_head(self, target):
        """Mueve la cabeza al ángulo objetivo.

        Argumentos:
            target (int):
                El ángulo objetivo en grados. 0 es la posición inicial,
                valores negativos están por debajo de este punto y valores positivos
                están por encima de este punto.
        """
        self.head_motor.run_target(20, target)

    def reset(self):
        # debe ser llamado cuando el cachorro está sentado.
        self.left_leg_motor.reset_angle(0)
        self.right_leg_motor.reset_angle(0)
        # Elige un número aleatorio de tiempo para acariciar al cachorro.
        self.acaricia_target = urandom.randint(3, 6)
        # Elige un número aleatorio de tiempo para alimentar al cachorro.
        self.alimentar_target = urandom.randint(2, 4)
        # El recuento de caricias y el recuento de alimentos comienzan en 1
        self.acaricia_count, self.alimentar_count = 1, 1
        # Reinicia los temporizadores.
        self.acaricia_count_timer.reset()
        self.alimentar_count_timer.reset()
        self.count_changed_timer.reset()
        # Establece el comportamiento inicial.
        self.behavior = self.idle

    # Los siguientes 8 métodos definen los 8 comportamientos del cachorro.

    def idle(self):
        """El cachorro esta ocioso y esperando a que alguien lo acaricie o le dé de comer""".
        if self.did_behavior_change:
            print('idle')
            self.stand_up()
        self.update_eyes()
        self.update_behavior()
        self.update_acaricia_count()
        self.update_alimentar_count()

    def go_to_sleep(self):
        """Hace que el cachorro se duerma."""
        if self.did_behavior_change:
            print('go_to_sleep')
            self.eyes = self.OJOS_CANSADOS
            self.sit_down()
            self.move_head(self.ANGULO_CABEZA_ABAJO)
            self.eyes = self.OJOS_DURMIENDO
            self.ev3.speaker.play_file(SoundFile.SNORING)
        if self.touch_sensor.pressed() and Button.CENTER in self.ev3.buttons.pressed():
            self.count_changed_timer.reset()
            self.behavior = self.wake_up

    def wake_up(self):
        """Hace que el cachorro se despierte."""
        if self.did_behavior_change:
            print('wake_up')
        self.eyes = self.OJOS_CANSADOS
        self.ev3.speaker.play_file(SoundFile.DOG_WHINE)
        self.move_head(self.ANGULO_DE_CABEZA_ARRIBA)
        self.sit_down()
        self.stretch()
        wait(1000)
        self.stand_up()
        self.behavior = self.idle

    def act_playful(self):
        """Hace que el cachorro actue de forma juguetona."""
        if self.did_behavior_change:
            print('act_playful')
            self.eyes = self.OJOS_NEUTRALES
            self.stand_up()
            self.playful_bark_interval = 0

        if self.update_acaricia_count():
            # Si el cachorro fue acariciado, entonces hemos terminado de ser juguetones
            self.behavior = self.idle

        if self.playful_timer.time() > self.playful_bark_interval:
            self.ev3.speaker.play_file(SoundFile.DOG_BARK_2)
            self.playful_timer.reset()
            self.playful_bark_interval = urandom.randint(4, 8) * 1000

    def act_angry(self):
        """Hace que el cachorro se enfade."""
        if self.did_behavior_change:
            print('act_angry')
        self.eyes = self.OJOS_ENOJADOS
        self.ev3.speaker.play_file(SoundFile.DOG_GROWL)
        self.stand_up()
        wait(1500)
        self.ev3.speaker.play_file(SoundFile.DOG_BARK_1)
        self.acaricia_count -= 1
        print('acaricia_count:', self.acaricia_count, 'acaricia_target:', self.acaricia_target)
        self.behavior = self.idle

    def act_hungry(self):
        if self.did_behavior_change:
            print('act_hungry')
            self.eyes = self.OJOS_HERIDOS
            self.sit_down()
            self.ev3.speaker.play_file(SoundFile.DOG_WHINE)

        if self.update_alimentar_count():
            # Si tenemos comida, entonces ya no tenemos hambre.
            self.behavior = self.idle

        if self.update_acaricia_count():
            # Si nos dan una mascota en vez de comida, entonces nos enfadamos.
            self.behavior = self.act_angry

    def go_to_bathroom(self):
        if self.did_behavior_change:
            print('go_to_bathroom')
        self.eyes = self.OJOS_BIZCOS
        self.stand_up()
        wait(100)
        self.right_leg_motor.run_target(100, self.ANGULO_DE_ESTIRAMIENTO)
        wait(800)
        self.ev3.speaker.play_file(SoundFile.HORN_1)
        wait(1000)
        for _ in range(3):
            self.right_leg_motor.run_angle(100, 20)
            self.right_leg_motor.run_angle(100, -20)
        self.right_leg_motor.run_target(100, self.ANGULO_DE_PIE)
        self.alimentar_count = 1
        self.behavior = self.idle

    def act_happy(self):
        if self.did_behavior_change:
            print('act_happy')
        self.eyes = self.OJOS_CORAZON
        # self.move_head(self.?)
        self.sit_down()
        for _ in range(3):
            self.ev3.speaker.play_file(SoundFile.DOG_BARK_1)
            self.hop()
        wait(500)
        self.sit_down()
        self.reset()

    def sit_down(self):
        """Hace que el cachorro se siente."""
        self.left_leg_motor.run(-50)
        self.right_leg_motor.run(-50)
        wait(1000)
        self.left_leg_motor.stop()
        self.right_leg_motor.stop()
        wait(100)

    # Los siguientes 4 métodos definen acciones que se utilizan para hacer algunas partes de
    # los comportamientos anteriores.

    def stand_up(self):
        """Hace que el cachorro se ponga de pie."""
        self.left_leg_motor.run_target(100, self.ANGULO_MEDIO_SUPERIOR, wait=False)
        self.right_leg_motor.run_target(100, self.ANGULO_MEDIO_SUPERIOR)
        while not self.left_leg_motor.control.done():
            wait(100)

        self.left_leg_motor.run_target(50, self.ANGULO_DE_PIE, wait=False)
        self.right_leg_motor.run_target(50, self.ANGULO_DE_PIE)
        while not self.left_leg_motor.control.done():
            wait(100)

        wait(500)

    def stretch(self):
        """Hace que el cachorro estire las patas hacia atrás."""
        self.stand_up()

        self.left_leg_motor.run_target(100, self.ANGULO_DE_ESTIRAMIENTO, wait=False)
        self.right_leg_motor.run_target(100, self.ANGULO_DE_ESTIRAMIENTO)
        while not self.left_leg_motor.control.done():
            wait(100)

        self.ev3.speaker.play_file(SoundFile.DOG_WHINE)

        self.left_leg_motor.run_target(100, self.ANGULO_DE_PIE, wait=False)
        self.right_leg_motor.run_target(100, self.ANGULO_DE_PIE)
        while not self.left_leg_motor.control.done():
            wait(100)

    def hop(self):
        """Hace que el cachorro salte."""
        self.left_leg_motor.run(500)
        self.right_leg_motor.run(500)
        wait(275)
        self.left_leg_motor.hold()
        self.right_leg_motor.hold()
        wait(275)
        self.left_leg_motor.run(-50)
        self.right_leg_motor.run(-50)
        wait(275)
        self.left_leg_motor.stop()
        self.right_leg_motor.stop()

    @property
    def behavior(self):
        """Obtiene y establece el comportamiento actual."""
        return self._behavior

    @behavior.setter
    def behavior(self, value):
        if self._behavior != value:
            self._behavior = value
            self._behavior_changed = True

    @property
    def did_behavior_change(self):
        """bool: Prueba si el comportamiento cambió desde la última vez que esta
        propiedad fue leída.
        """
        if self._behavior_changed:
            self._behavior_changed = False
            return True
        return False

    def update_behavior(self):
        """Actualiza la propiedad :prop:`behavior` basándose en el estado actual
        de acariciar y alimentar.
        """
        if self.acaricia_count == self.acaricia_target and self.alimentar_count == self.alimentar_target:
            # Si tenemos la cantidad exacta de caricias y alimentos, actuar feliz.
            self.behavior = self.act_happy
        elif self.acaricia_count > self.acaricia_target and self.alimentar_count < self.alimentar_target:
            # Si tenemos demasiadas caricias y poca comida, actúa con rabia.
            self.behavior = self.act_angry
        elif self.acaricia_count < self.acaricia_target and self.alimentar_count > self.alimentar_target:
            # Si no tenemos suficientes caricias y demasiada comida, vamos al baño.
            self.behavior = self.go_to_bathroom
        elif self.acaricia_count == 0 and self.alimentar_count > 0:
            # Si no tenemos caricias y algo de comida, actúa juguetón.
            self.behavior = self.act_playful
        elif self.alimentar_count == 0:
            # Si no tenemos comida, actúa con hambre.
            self.behavior = self.act_hungry

    @property
    def eyes(self):
        """Consigue y fija la mirada""".
        return self._eyes

    @eyes.setter
    def eyes(self, value):
        if value != self._eyes:
            self._eyes = value
            self.ev3.screen.load_image(value)

    def update_eyes(self):
        if self.eyes_timer_1.time() > self.eyes_timer_1_end:
            self.eyes_timer_1.reset()
            if self.eyes == self.OJOS_DURMIENDO:
                self.eyes_timer_1_end = urandom.randint(1, 5) * 1000
                self.eyes = self.OJO_DERECHO_CANSADO
            else:
                self.eyes_timer_1_end = 250
                self.eyes = self.OJOS_DURMIENDO

        if self.eyes_timer_2.time() > self.eyes_timer_2_end:
            self.eyes_timer_2.reset()
            if self.eyes != self.OJOS_DURMIENDO:
                self.eyes_timer_2_end = urandom.randint(1, 10) * 1000
                if self.eyes != self.OJO_IZQUIERDO_CANSADO:
                    self.eyes = self.OJO_IZQUIERDO_CANSADO
                else:
                    self.eyes = self.OJO_DERECHO_CANSADO

    def update_acaricia_count(self):
        """Actualiza el atributo :attr:`acaricia_count` si el cachorro está siendo
        acariciado (sensor táctil pulsado).

        Devuelve:
            bool:
                ``True`` si el cachorro fue acariciado desde la última vez que este método fue llamado.
                 en caso contrario ``False``.
        """
        changed = False

        petted = self.touch_sensor.pressed()
        if petted and petted != self.prev_petted:
            self.acaricia_count += 1
            print('acaricia_count:', self.acaricia_count, 'acaricia_target:', self.acaricia_target)
            self.count_changed_timer.reset()
            if not self.behavior == self.act_hungry:
                self.eyes = self.OJOS_BIZCOS
                self.ev3.speaker.play_file(SoundFile.DOG_SNIFF)
            changed = True

        self.prev_petted = petted
        return changed

    def update_alimentar_count(self):
        """Actualiza el atributo :attr:`alimentar_count` si el cachorro está siendo
        alimentado (el sensor de color detecta un color).

        Devuelve:
            bool:
                ``True`` si el cachorro fue alimentado desde la última vez que este método fue llamado.
                 de lo contrario ``False``.
        """
        color = self.color_sensor.color()
        changed = False

        if color is not None and color != Color.BLACK and color != self.prev_color:
            self.alimentar_count += 1
            print('alimentar_count:', self.alimentar_count, 'alimentar_target:', self.alimentar_target)
            changed = True
            self.count_changed_timer.reset()
            self.prev_color = color
            self.eyes = self.OJOS_BIZCOS
            self.ev3.speaker.play_file(SoundFile.CRUNCHING)

        return changed

    def monitor_counts(self):
        """Supervisa los recuentos de caricias y alimentos y los reduce con el tiempo"""
        if self.acaricia_count_timer.time() > 15000:
            self.acaricia_count_timer.reset()
            self.acaricia_count = max(0, self.acaricia_count - 1)
            print('acaricia_count:', self.acaricia_count, 'acaricia_target:', self.acaricia_target)
        if self.alimentar_count_timer.time() > 15000:
            self.alimentar_count_timer.reset()
            self.alimentar_count = max(0, self.alimentar_count - 1)
            print('alimentar_count:', self.alimentar_count, 'alimentar_target:', self.alimentar_target)
        if self.count_changed_timer.time() > 30000:
            # Si no ha pasado nada durante 30 segundos, ir a dormir
            self.count_changed_timer.reset()
            self.behavior = self.go_to_sleep

    def run(self):
        """Este es el bucle de ejecución del programa principal."""
        self.sit_down()
        self.adjust_head()
        self.eyes = self.OJOS_DURMIENDO
        self.reset()
        while True:
            self.monitor_counts()
            self.behavior()
            wait(100)


# Esto cubre la lágrima para hacer una nueva imagen.
Puppy.OJOS_BIZCOS.draw_box(120, 60, 140, 85, fill=True, color=Color.WHITE)


if __name__ == '__main__':
    puppy = Puppy()
    puppy.run()
