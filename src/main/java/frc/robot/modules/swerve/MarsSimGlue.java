// Copiado de MARS (sim/glue/java/MarsSimGlue.java). Cambios: la ruta de TunerConstants y la
// bandera MARS_SIM (ver habilitado()).
//
// Que es
// ======
// Cambia el motor de simulacion del swerve. CTRE simula los modulos con
// cinematica: integra la pose a partir de las velocidades que pide el codigo.
// No hay friccion, ni masa, ni colisiones --- el robot atraviesa las paredes.
//
// Esta clase sustituye esa simulacion por la de Gazebo. El robot pasa a tener
// masa, las ruedas patinan, y si choca contra algo se para.
//
// Lo que NO cambia
// ================
// NADA de tu logica de swerve. TunerConstants, SwerveRequestFactory,
// DriverBindings, las ganancias de Slot0Configs: todo igual. No hay una segunda
// implementacion del swerve en ningun sitio. Lo unico que cambia es de donde
// salen los valores de los encoders.
//
// Y si el Studio no esta corriendo, esta clase se aparta sola y deja que CTRE
// simule como siempre. Trabajar sin la simulacion no requiere tocar nada.
//
// Bandera
// =======
// Por default NO se crea: `simulateJava` es la simulacion normal de WPILib + CTRE.
// Se activa con `./gradlew simulateJava -Pmarssim` (o la variable de entorno
// MARS_SIM=1, para el boton "Simulate Robot Code" de VS Code).
//
// Usa NetworkIO de ForgeMini para todo el trafico de NT, no la API cruda: es la
// convencion del proyecto y ya esta probada en el.

package frc.robot.modules.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.stzteam.forgemini.io.NetworkIO;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.configuration.constants.swerve.TunerConstants;

public final class MarsSimGlue {
  /** Tablas del contrato. Ver sim/protocol/topics.toml, seccion [[glue]]. */
  private static final String SALIDA = "MARS/Glue/out";
  private static final String ENTRADA = "MARS/Glue/in";
  /** Diagnostico del propio glue: dice si esta clase vive y si su hilo corre. */
  private static final String DIAG = "MARS/Glue/diag";

  private static final double DOS_PI = 2.0 * Math.PI;

  /** El orden TIENE que ser el del robot-map, y el de createDrivetrain(). */
  private static final String[] MODULOS = {"fl", "fr", "bl", "br"};

  /**
   * Ciclos sin latido nuevo antes de dar la simulacion por ausente. A 250 Hz
   * son 0.4 s: bastante para no parpadear con un mensaje tardio, poco para que
   * abrir el Studio a mitad de sesion se note enseguida.
   */
  private static final int CICLOS_PARA_DARLA_POR_MUERTA = 100;

  private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
  private final double reduccionDireccion;
  private final double acoplamiento;

  private double ultimoInstante = Utils.getCurrentTimeSeconds();
  private double ultimoLatido = -1.0;
  private int ciclosSinLatido = Integer.MAX_VALUE;
  private boolean conectado = false;
  private boolean reventado = false;
  private long ticks = 0;

  /** Si se pidio usar la fisica de MARS (bandera -Pmarssim o MARS_SIM=1). */
  public static boolean habilitado() {
    String v = System.getenv("MARS_SIM");
    return Boolean.getBoolean("mars.sim") || "1".equals(v) || "true".equalsIgnoreCase(v);
  }

  public MarsSimGlue(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
    this.drivetrain = drivetrain;

    // Se leen de TunerConstants en vez de escribirlas otra vez: si alguien
    // cambia la reduccion alli, esto la sigue sin que nadie se acuerde.
    this.reduccionDireccion = TunerConstants.FrontLeft.SteerMotorGearRatio;
    this.acoplamiento = TunerConstants.FrontLeft.CouplingGearRatio;

    // Dos senales de vida --- consola y NetworkTables --- porque no siempre se
    // esta mirando la consola. Si `MARS/Glue/diag/creado` no aparece en el
    // arbol de NT, esta clase no llego a construirse, y entonces el problema
    // esta en el `if` que decide crearla y no aqui dentro.
    System.out.println("[MARS] glue creado, publicando en /" + SALIDA + "/");
    NetworkIO.set(DIAG, "creado", true);
    NetworkIO.set(DIAG, "reduccionDireccion", reduccionDireccion);
    NetworkIO.set(DIAG, "acoplamiento", acoplamiento);
  }

  /** Si la fisica de MARS esta alimentando al robot ahora mismo. */
  public boolean conectado() {
    return conectado;
  }

  /**
   * Sustituye a {@code updateSimState}. Llamar desde el hilo de simulacion del
   * drivetrain, a la misma tasa a la que se llamaba a aquel.
   */
  public void update() {
    // Un Notifier cuyo callback lanza se muere SIN DECIR NADA: el robot se
    // queda quieto y no hay ni una linea en la consola. Atrapar y avisar una
    // vez convierte ese fallo mudo en uno que se puede leer.
    try {
      actualizar();
    } catch (Throwable e) {
      if (!reventado) {
        reventado = true;
        System.err.println("[MARS] el glue fallo y se apaga: " + e);
        e.printStackTrace();
        NetworkIO.set(DIAG, "error", e.toString());
      }
    }
  }

  private void actualizar() {
    final double ahora = Utils.getCurrentTimeSeconds();
    final double dt = ahora - ultimoInstante;
    ultimoInstante = ahora;

    // Contador de vida del hilo. Si `MARS/Glue/diag/ticks` no sube en el arbol
    // de NT, este hilo no esta corriendo --- que es un problema distinto de que
    // la simulacion no conteste, y se arregla en otro sitio.
    NetworkIO.set(DIAG, "ticks", (double) (++ticks));

    final double bus = RobotController.getBatteryVoltage();
    NetworkIO.set(SALIDA, "enabled", DriverStation.isEnabled());

    // Publicar SIEMPRE, incluso desconectado: es lo que hace que arrancar MARS
    // Sim a mitad de sesion funcione sin reiniciar el robot.
    publicarMandos(bus);

    if (!hayDatos()) {
      // Sin simulacion, la de CTRE. Trabajar sin abrir el Studio queda
      // exactamente como antes de instalar nada de esto.
      if (conectado) {
        conectado = false;
        System.out.println("[MARS] simulacion ausente; volviendo a la de CTRE");
        NetworkIO.set(DIAG, "conectado", false);
      }
      drivetrain.updateSimState(dt, bus);
      return;
    }

    if (!conectado) {
      conectado = true;
      System.out.println("[MARS] simulacion conectada; la fisica manda");
      NetworkIO.set(DIAG, "conectado", true);
    }
    inyectarFisica(bus);
  }

  /** Lo que el codigo del robot le esta pidiendo a cada motor. */
  private void publicarMandos(double bus) {
    var estado = drivetrain.getState();

    for (int i = 0; i < 4; i++) {
      var modulo = drivetrain.getModule(i);
      var traccion = modulo.getDriveMotor().getSimState();
      var direccion = modulo.getSteerMotor().getSimState();

      // Phoenix necesita saber el voltaje de bus antes de poder decir que esta
      // aplicando. Sin esto getMotorVoltage() devuelve cero y el robot no se
      // mueve, sin ningun error en ninguna parte.
      traccion.setSupplyVoltage(bus);
      direccion.setSupplyVoltage(bus);

      NetworkIO.set(SALIDA, MODULOS[i] + "_drive/duty", traccion.getMotorVoltage() / bus);

      // El angulo objetivo del modulo, que es lo que tu swerve ya calculo --- y
      // ya optimizado, o sea que nunca pide girar mas de 90 grados.
      if (estado.ModuleTargets != null && estado.ModuleTargets.length > i) {
        double vueltasModulo = estado.ModuleTargets[i].angle.getRotations();
        NetworkIO.set(
            SALIDA, MODULOS[i] + "_steer/setpoint", vueltasModulo * DOS_PI * reduccionDireccion);
      }
    }
  }

  /** Lo que la fisica devolvio, metido en los sensores del robot. */
  private void inyectarFisica(double bus) {
    for (int i = 0; i < 4; i++) {
      var modulo = drivetrain.getModule(i);
      var traccion = modulo.getDriveMotor().getSimState();
      var direccion = modulo.getSteerMotor().getSimState();
      var encoder = modulo.getEncoder().getSimState();
      encoder.setSupplyVoltage(bus);

      // El contrato entrega radianes y rad/s del eje del MOTOR; Phoenix quiere
      // vueltas y vueltas por segundo del ROTOR, que es el mismo eje.
      double vueltasDireccion =
          NetworkIO.get(ENTRADA, MODULOS[i] + "_steer/position", 0.0) / DOS_PI;
      double vueltasDireccionSeg =
          NetworkIO.get(ENTRADA, MODULOS[i] + "_steer/velocity", 0.0) / DOS_PI;
      double vueltasModulo = vueltasDireccion / reduccionDireccion;
      double vueltasModuloSeg = vueltasDireccionSeg / reduccionDireccion;

      direccion.setRawRotorPosition(Rotations.of(vueltasDireccion));
      direccion.setRotorVelocity(RotationsPerSecond.of(vueltasDireccionSeg));

      // El CANcoder mide el MODULO, no el rotor: hay que deshacer la reduccion.
      // Y hay que alimentarlo aunque parezca redundante, porque la direccion
      // usa FusedCANcoder: si el CANcoder no se mueve, el control cree que el
      // modulo esta clavado y satura.
      encoder.setRawPosition(Rotations.of(vueltasModulo));
      encoder.setVelocity(RotationsPerSecond.of(vueltasModuloSeg));

      // ACOPLAMIENTO. En un MK4, girar el azimut arrastra el motor de traccion:
      // kCoupleRatio vueltas de motor por vuelta de modulo. Gazebo no lo modela
      // --- sus dos joints son independientes --- asi que se suma aqui, que es
      // donde se convierte a unidades de rotor.
      //
      // Sin esto el encoder de traccion no se moveria al girar los modulos, y
      // la odometria del robot simulado seria MAS limpia que la del real justo
      // en la maniobra donde peor se porta.
      double vueltasTraccion =
          NetworkIO.get(ENTRADA, MODULOS[i] + "_drive/position", 0.0) / DOS_PI
              + vueltasModulo * acoplamiento;
      double vueltasTraccionSeg =
          NetworkIO.get(ENTRADA, MODULOS[i] + "_drive/velocity", 0.0) / DOS_PI
              + vueltasModuloSeg * acoplamiento;

      traccion.setRawRotorPosition(Rotations.of(vueltasTraccion));
      traccion.setRotorVelocity(RotationsPerSecond.of(vueltasTraccionSeg));
    }

    // El giroscopio sale de la fisica, no de integrar las velocidades. Esa es
    // justamente la diferencia: si la odometria se equivoca, aqui se nota.
    drivetrain
        .getPigeon2()
        .getSimState()
        .setRawYaw(Degrees.of(NetworkIO.get(ENTRADA, "gyro/yaw", 0.0)));
  }

  /**
   * Si la simulacion esta publicando de verdad.
   *
   * Se mira el LATIDO --- un contador que el bridge sube en cada ciclo --- y no
   * el valor de los sensores: un robot parado publica ceros que no se
   * distinguen de un canal muerto, y con eso el robot se quedaria congelado al
   * arrancar sin que nada lo explicara.
   */
  private boolean hayDatos() {
    double latido = NetworkIO.get(ENTRADA, "heartbeat", -1.0);
    if (latido != ultimoLatido) {
      ultimoLatido = latido;
      ciclosSinLatido = 0;
      return true;
    }
    if (ciclosSinLatido < CICLOS_PARA_DARLA_POR_MUERTA) {
      ciclosSinLatido++;
      return true;
    }
    return false;
  }
}
