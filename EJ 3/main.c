// Incluyo librería estándar para poder utilizar printf()
#include <stdio.h>

//librería matemática para utilizar atan2f() sqrtf() y sinf()
#include <math.h>

// Tiempo entre cada iteración del programa (usaremos 0,1 sg)
#define DT 0.1f

// Definir el valor de PI para realizar cálculos trigonométricos
#define PI 3.14159265359f


// ESTRUCTURA DE DATOS DEL IMU

// Estructura para almacenar los datos del ICM-42688-P
typedef struct
{
    float ax;       // Aceleración en el eje X
    float ay;       // Acl en el eje Y
    float az;       // Acl en el eje Z

    float gx;       // Velocidad angular alrededor del eje X
    float gy;       // Vang alrededor del eje Y
    float gz;       // Vang alrededor del eje Z

} IMU_Data;


// ESTRUCTURA DE DATOS DEL BARÓMETRO

// Crear estructura para almacenar los datos del MS5607
typedef struct
{
    float altitude;     // Altitud q se obtiene a partir de la presión

} Barometer_Data;


// ESTRUCTURA DE DATOS DEL GNSS

// Estructura para almacenar los datos del MAX-M10S
typedef struct
{
    float altitude;     // Altitud dada por el GNSS
    float velocity;     // Velocidad vertical dada por el GNSS

} GNSS_Data;


// ESTRUCTURA DEL FILTRO COMPLEMENTARIO

// Esta estructura almacena la estimación de orientación
typedef struct
{
    float roll;         // Ángulo de giro alrededor del eje X
    float pitch;        // Ángulo de giro alrededor del eje Y

} ComplementaryFilter;


// ESTRUCTURA DEL FILTRO DE KALMAN

// Esta estructura almacena el estado y la incertidumbre del filtro
typedef struct
{
    float altitude;     // Altitud estimada por el filtro
    float velocity;     // Velocidad vertical estimada

    float P00;          // Incertidumbre asociada a la alt
    float P01;          // Covarianza entre alt y vel
    float P10;          // Covarianza entre vel y alt
    float P11;          // Incertidumbre asociada a la vel

} KalmanFilter;


// INICIALIZACIÓN DEL FILTRO COMPLEMENTARIO

void ComplementaryFilter_Init(ComplementaryFilter *filter)
{
    // Inicializo el roll a 0 grados
    filter->roll = 0.0f;

    // Inicializo el pitch a 0 grados
    filter->pitch = 0.0f;
}


// ACTUALIZACIÓN DEL FILTRO COMPLEMENTARIO

void ComplementaryFilter_Update(
    ComplementaryFilter *filter,  // Filtro que vamos a actualizar
    IMU_Data *imu,                 // Datos procedentes del IMU
    float dt)                      // Tiempo q pasa
{
    // Variables donde se calcula la orientación usando el acelerómetro
    float roll_acc;
    float pitch_acc;

    // Peso que damos a los datos del giroscopio
    float alpha = 0.98f;


    // CALCULAMOS EL ROLL CON EL ACELERÓMETRO

    // atan2f() para calcular el ángulo a partir de las
    // aceleraciones de los ejes Y y Z
    roll_acc =
        atan2f(imu->ay, imu->az) * 180.0f / PI;


    // CALCULAR  PITCH CON EL ACELERÓMETRO

    // Calculo el pitch con las aceleraciones X, Y y Z
    pitch_acc =
        atan2f(
            -imu->ax,
            sqrtf(
                imu->ay * imu->ay +
                imu->az * imu->az
            )
        ) * 180.0f / PI;


    // PREDICCIÓN CON EL GIROSCOPIO

    // Integramos la vang para obtener el cambio
    // de orientación durante el intervalo de tiempo dt
    filter->roll += imu->gx * dt;

    // Hacer lo mismo para el pitch
    filter->pitch += imu->gy * dt;


    // FUSIÓN DEL GIROSCOPIO Y ACELERÓMETRO

    // Damos un 98 % de peso a la estimación del giroscopio
    // y un 2 % a la estimación del acelerómetro
    filter->roll =
        alpha * filter->roll +
        (1.0f - alpha) * roll_acc;


    // Hacemos lo mismo para el pitch
    filter->pitch =
        alpha * filter->pitch +
        (1.0f - alpha) * pitch_acc;
}


// INICIALIZACIÓN DEL FILTRO DE KALMAN

void Kalman_Init(
    KalmanFilter *filter,       // Filtro a inicializar
    float initial_altitude)     // Alt inicial
{
    // Establezco altitud inicial
    filter->altitude = initial_altitude;

    // Voy a suponer q el cohete está parado al principio
    filter->velocity = 0.0f;


    // INCERTIDUMBRES INICIALES

    // Incertidumbre inicial de la altitud
    filter->P00 = 10.0f;

    // Covarianza inicial entre alt y vel
    filter->P01 = 0.0f;

    // Covarianza inicial entre vel y alt
    filter->P10 = 0.0f;

    // Incertidumbre inicial de la vel
    filter->P11 = 10.0f;
}


// PREDICCIÓN DEL FILTRO DE KALMAN

void Kalman_Predict(
    KalmanFilter *filter,       // Filtro que estamos usando
    float acceleration,          // Aceleración medida
    float dt)                    // Tiempo qu ha pasado
{
    // PREDICCIÓN DE LA ALTITUD

    // Utilizamos la ecuación básica de movimiento
    //
    // x = x0 + v*t + 1/2*a*t²
    //
    // para estimar dónde estará el cohete
    filter->altitude +=
        filter->velocity * dt +
        0.5f * acceleration * dt * dt;


    // PREDICCIÓN DE LA VELOCIDAD

    // Utilizo
    //
    // v = v0 + a*t
    //
    // para actualizar la velocidad estimada
    filter->velocity +=
        acceleration * dt;


    // RUIDO DEL MODELO

    // Incertidumbre que añadimos a la estimación de alt
    float Q_altitude = 0.05f;

    // Incertidumbre que añadimos a la estimación de vel
    float Q_velocity = 0.5f;


    // Actualizamos la incertidumbre de la altitud
    filter->P00 +=
        dt * dt * filter->P11 +
        Q_altitude;


    // Actualizae la incertidumbre de la velocidad
    filter->P11 += Q_velocity;
}


// CORRECCIÓN DEL KALMAN CON EL BARÓMETRO

void Kalman_UpdateBarometer(
    KalmanFilter *filter,        // Filtro de Kalman
    float measured_altitude)     // Alt medida por el MS5607
{
    // R -->incertidumbre que atribuimos
    // a la medida del barómetro
    float R = 4.0f;


    // CALCULAMOS EL ERROR DE LA MEDIDA

    // Comparar la altitud medida por el barómetro
    // con nuestra altitud estimada
    float innovation =
        measured_altitude -
        filter->altitude;


    // CALCULAMOS LA GANANCIA DE KALMAN

    // La ganancia determina cuánto confiamos en el barómetro
    // frente a nuestra estimación anterior
    float K =
        filter->P00 /
        (filter->P00 + R);


    // CORREGIMOS LA ALTITUD

    // Añadimos una parte del error a nuestra estimación
    filter->altitude +=
        K * innovation;


    // ACTUALIZAMOS LA INCERTIDUMBRE

    // Después de utilizar el barómetro
    // reducimos la incertidumbre de la estimación
    filter->P00 =
        (1.0f - K) *
        filter->P00;
}


// CORRECCIÓN DEL KALMAN CON EL GNSS

void Kalman_UpdateGNSS(
    KalmanFilter *filter,        // Filtro de Kalman
    float measured_altitude,     // Alt medida por GNSS
    float measured_velocity)     // Vel medida por GNSS
{
    // Incertidumbre que asignaremos a la altitud del GNSS
    float R_altitude = 25.0f;

    // Incertidumbre que asignaremos a la velocidad del GNSS
    float R_velocity = 4.0f;


    // CORRECCIÓN DE LA ALTITUD

    // Cálculo de diferencia entre la medida del GNSS
    // y nuestra estimación
    float innovation_altitude =
        measured_altitude -
        filter->altitude;


    // Cálculo de cuánto debemos confiar en el GNSS
    float K_altitude =
        filter->P00 /
        (filter->P00 + R_altitude);


    // Corregimos la altitud estimada
    filter->altitude +=
        K_altitude *
        innovation_altitude;


    // Actualizamos la incertidumbre de la altitud
    filter->P00 =
        (1.0f - K_altitude) *
        filter->P00;


    // CORRECCIÓN DE LA VELOCIDAD

    // Calculamos el error de la velocidad
    float innovation_velocity =
        measured_velocity -
        filter->velocity;


    // Calculamos cuánto confiamos en la velocidad del GNSS
    float K_velocity =
        filter->P11 /
        (filter->P11 + R_velocity);


    // Corregimos la velocidad estimada
    filter->velocity +=
        K_velocity *
        innovation_velocity;


    // Actualizamos la incertidumbre de la velocidad
    filter->P11 =
        (1.0f - K_velocity) *
        filter->P11;
}


// SIMULACIÓN DEL ICM-42688-P

IMU_Data SimulateIMU(float time)
{
    // Variable para almacenar los datos simulados
    IMU_Data imu;


    // SIMULANDO LA ACELERACIÓN DEL COHETE

    // Durante los primeros 5 segundos simulamos
    // una aceleración elevada
    if (time < 5.0f)
        imu.az = 30.0f;


    // Entre los 5 y los 10 segundos reducimos la aceleración
    else if (time < 10.0f)
        imu.az = 15.0f;


    // Después de los 10 segundos suponemos aceleración cero
    else
        imu.az = 0.0f;


    // No añadimos aceleración en X
    imu.ax = 0.0f;

    // No añadimos aceleración en Y
    imu.ay = 0.0f;


    // SIMULANDO EL GIROSCOPIO

    // No simulamos rotación alrededor de X
    imu.gx = 0.0f;

    // No simulamos rotación alrededor de Y
    imu.gy = 0.0f;

    // No simulamos rotación alrededor de Z
    imu.gz = 0.0f;


    // Devolver todos los datos simulados
    return imu;
}


// SIMULACIÓN DEL MS5607

Barometer_Data SimulateBarometer(float real_altitude)
{
    // Variable para almacenar los datos simulados
    Barometer_Data barometer;


    // Simulamos una medida del barómetro.
    //
    // Variación sinusoidal para representar
    // el ruido o error de medida del sensor
    barometer.altitude =
        real_altitude +
        2.0f * sinf(real_altitude * 0.01f);


    // Devolver valor simulado
    return barometer;
}


// SIMULACIÓN DEL MAX-M10S

GNSS_Data SimulateGNSS(
    float real_altitude,       // Alt real
    float real_velocity)       // Vel real
{
    // Variable para almacenar los datos simulados
    GNSS_Data gnss;


    // Simulamos la medida de altitud del GNSS
    //
    // Añadimos una variación para representar
    // el error de la medición
    gnss.altitude =
        real_altitude +
        5.0f * sinf(real_altitude * 0.005f);


    // Simulamos la velocidad medida por el GNSS
    //
    // También añadimos un pequeño error
    gnss.velocity =
        real_velocity +
        1.0f * sinf(real_velocity * 0.1f);


    // Devolver datos simulados
    return gnss;
}


// FUNCIÓN PRINCIPAL

int main(void)
{
    // Creamos el filtro que estimará la orientación
    ComplementaryFilter orientationFilter;


    // Creamos el filtro que estimará alt y vel
    KalmanFilter navigationFilter;


    // Inicializamos el filtro complementario
    ComplementaryFilter_Init(
        &orientationFilter
    );


    // Inicializamos el filtro de Kalman
    //
    // Supongo que el cohete comienza a 0 m
    Kalman_Init(
        &navigationFilter,
        0.0f
    );


    // VARIABLES QUE REPRESENTAN LA REALIDAD SIMULADA

    // Altitud real del cohete
    float real_altitude = 0.0f;

    // Velocidad real del cohete
    float real_velocity = 0.0f;


    // BUCLE PRINCIPAL DE LA SIMULACIÓN

    // Se van a ejecutar 200 iteraciones
    //
    // Como cada iteración representa 0,1 segundos,
    // simulamos un total de 20 segundos
    for (int i = 0; i < 200; i++)
    {
        //Cálculo de tiempo transcurrido
        float time = i * DT;


        // OBTENCIÓN DE LOS DATOS SIMULADOS DEL IMU

        // Simulamos lo que mediría el ICM-42688-P
        IMU_Data imu =
            SimulateIMU(time);


        // CALCULAMOS LA TRAYECTORIA REAL

        // Actualizamos la altitud real usando
        //
        // x = x0 + v*t + 1/2*a*t²
        real_altitude +=
            real_velocity * DT +
            0.5f * imu.az * DT * DT;


        // Actualizamos la velocidad real usanod:
        //
        // v = v0 + a*t
        real_velocity +=
            imu.az * DT;


        // SIMULANDO EL BARÓMETRO

        // Obtenemos la medida simulada del MS5607
        Barometer_Data barometer =
            SimulateBarometer(real_altitude);


        // SIMULANDO EL GNSS

        // Obtenemos las medidas simuladas del MAX-M10S
        GNSS_Data gnss =
            SimulateGNSS(
                real_altitude,
                real_velocity
            );


        // FILTRO COMPLEMENTARIO

        // Fusión acelerómetro y giroscopio
        // para estimar roll y pitch
        ComplementaryFilter_Update(
            &orientationFilter,
            &imu,
            DT
        );


        // PREDICCIÓN DEL FILTRO DE KALMAN

        // Uso la aceleración del IMU para predecir
        // la nueva altitud y velocidad
        Kalman_Predict(
            &navigationFilter,
            imu.az,
            DT
        );


        // CORRECCIÓN CON EL BARÓMETRO

        // Utilizamos la altitud del MS5607 para corregir
        // la estimación
        Kalman_UpdateBarometer(
            &navigationFilter,
            barometer.altitude
        );


        // CORRECCIÓN CON EL GNSS

        // Utilizamos la alt y vel del GNSS
        // para realizar otra corrección
        Kalman_UpdateGNSS(
            &navigationFilter,
            gnss.altitude,
            gnss.velocity
        );


        // MOSTRAR LOS RESULTADOS

        // Imprimimos en pantalla:
        //
        // - Tiempo
        // - Altitud real
        // - Altitud estimada
        // - Velocidad estimada
        // - Roll
        // - Pitch
        printf(
            "t = %.1f s | "
            "Alt real = %.2f m | "
            "Alt estimada = %.2f m | "
            "Vel estimada = %.2f m/s | "
            "Roll = %.2f | "
            "Pitch = %.2f\n",

            time,

            real_altitude,

            navigationFilter.altitude,

            navigationFilter.velocity,

            orientationFilter.roll,

            orientationFilter.pitch
        );
    }


    // Para indicar que el programa acabó correctamente :)
    return 0;
}