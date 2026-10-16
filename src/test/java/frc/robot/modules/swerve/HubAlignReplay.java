// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.stzteam.features.robotlocalization.fusion.PoseFusion;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Graba los recorridos de las pruebas de HubAlign y genera un visor HTML
 * (build/reports/hubalign/hubalign.html). Cada clase de prueba corre en su propio proceso, asi que
 * cada una escribe su JSON y regenera el HTML con todos los JSON que encuentre.
 */
final class HubAlignReplay {

  static final Path DIR = Path.of("build", "reports", "hubalign");
  private static final List<Map<String, Object>> RUNS = new ArrayList<>();
  private static Map<String, Object> current;
  private static List<double[]> est;
  private static List<double[]> truth;
  private static List<Map<String, Object>> events;
  private static long t0;
  private static String lastStatus;
  private static List<Map<String, Object>> plans;

  /** Registra en el recorrido actual cada ruta que PathPlanner activa (inicial y replaneos). */
  static void installPathLogger() {
    com.pathplanner.lib.util.PathPlannerLogging.setLogActivePathCallback(
        poses -> {
          if (current == null || poses == null || poses.isEmpty()) {
            return;
          }
          List<double[]> pts = new ArrayList<>();
          for (Pose2d p : poses) {
            pts.add(new double[] {r(p.getX()), r(p.getY())});
          }
          Map<String, Object> plan = new LinkedHashMap<>();
          plan.put("t", r((System.nanoTime() - t0) / 1e9));
          plan.put("points", pts);
          plans.add(plan);
        });
  }

  private HubAlignReplay() {}

  static void begin(String name, Alliance alliance, String spot, Pose2d start, Pose2d target) {
    current = new LinkedHashMap<>();
    current.put("name", name);
    current.put("alliance", alliance.toString());
    current.put("spot", spot);
    current.put("start", pose(start));
    current.put("target", pose(target));
    est = new ArrayList<>();
    truth = null;
    events = new ArrayList<>();
    plans = new ArrayList<>();
    current.put("plans", plans);
    current.put("est", est);
    current.put("events", events);
    t0 = System.nanoTime();
    lastStatus = null;
  }

  /** Muestra de la pose estimada (y la real, si la prueba la conoce). Llamar cada ciclo. */
  static void sample(Pose2d estimated, Pose2d real) {
    if (current == null) {
      return;
    }
    double t = (System.nanoTime() - t0) / 1e9;
    est.add(new double[] {r(t), r(estimated.getX()), r(estimated.getY()), r(estimated.getRotation().getRadians())});
    if (real != null) {
      if (truth == null) {
        truth = new ArrayList<>();
        current.put("truth", truth);
      }
      truth.add(new double[] {r(t), r(real.getX()), r(real.getY()), r(real.getRotation().getRadians())});
    }
    String status = SmartDashboard.getString("HubAlign/estado", "");
    if (!status.equals(lastStatus)) {
      Map<String, Object> ev = new LinkedHashMap<>();
      ev.put("t", r(t));
      ev.put("text", status);
      ev.put("x", r(estimated.getX()));
      ev.put("y", r(estimated.getY()));
      events.add(ev);
      lastStatus = status;
    }
  }

  static void end(boolean passed, double seconds, double errCm, double errDeg, String note) {
    if (current == null) {
      return;
    }
    current.put("passed", passed);
    current.put("seconds", r(seconds));
    current.put("errCm", r(errCm));
    current.put("errDeg", r(errDeg));
    current.put("status", SmartDashboard.getString("HubAlign/estado", ""));
    // La variante resuelta (p. ej. cual fue "la mas cercana") la publica el comando al empezar.
    current.put("spot", SmartDashboard.getString("HubAlign/posicion", (String) current.get("spot")));
    current.put("replans", SmartDashboard.getNumber("HubAlign/replaneos", 0));
    current.put("note", note);
    RUNS.add(current);
    current = null;
  }

  /** Escribe el JSON de esta clase de prueba y regenera el visor con todas. */
  static void write(String testClass) {
    try {
      Files.createDirectories(DIR);
      ObjectMapper json = new ObjectMapper();
      json.writeValue(DIR.resolve(testClass + ".runs.json").toFile(), RUNS);

      List<Object> all = new ArrayList<>();
      File[] files = DIR.toFile().listFiles((d, n) -> n.endsWith(".runs.json"));
      if (files != null) {
        java.util.Arrays.sort(files);
        for (File f : files) {
          all.addAll(json.readValue(f, List.class));
        }
      }
      Map<String, Object> data = new LinkedHashMap<>();
      data.put("field", field(json));
      data.put("runs", all);
      data.put("generated", java.time.LocalDateTime.now().withNano(0).toString());

      String template;
      try (var in = HubAlignReplay.class.getResourceAsStream("/hubalign-viewer.html")) {
        template = new String(in.readAllBytes(), StandardCharsets.UTF_8);
      }
      String fragment = template.replace("/*__DATA__*/null", json.writeValueAsString(data));
      Files.writeString(DIR.resolve("hubalign-fragment.html"), fragment, StandardCharsets.UTF_8);
      Files.writeString(
          DIR.resolve("hubalign.html"),
          "<!doctype html>\n<html lang=\"es\"><head><meta charset=\"utf-8\">"
              + "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"></head><body>\n"
              + fragment
              + "\n</body></html>\n",
          StandardCharsets.UTF_8);
      System.out.println("[replay] " + DIR.resolve("hubalign.html").toAbsolutePath() + " (" + all.size() + " recorridos)");
    } catch (IOException e) {
      throw new RuntimeException("no se pudo escribir el visor de HubAlign", e);
    }
  }

  /** Geometria fija del campo para dibujar: tamano, navgrid, tags y posiciones de tiro. */
  private static Map<String, Object> field(ObjectMapper json) throws IOException {
    AprilTagFieldLayout layout = PoseFusion.defaultFieldLayout();
    Map<String, Object> f = new LinkedHashMap<>();
    f.put("length", layout.getFieldLength());
    f.put("width", layout.getFieldWidth());
    f.put("bumper", 0.9017);
    f.put("navgrid", json.readValue(new File("src/main/deploy/pathplanner/navgrid.json"), Map.class));
    List<Map<String, Object>> tags = new ArrayList<>();
    for (AprilTag tag : layout.getTags()) {
      Pose2d p = tag.pose.toPose2d();
      tags.add(Map.of("id", tag.ID, "x", r(p.getX()), "y", r(p.getY()), "yaw", r(p.getRotation().getRadians())));
    }
    f.put("tags", tags);
    Map<String, Object> spots = new LinkedHashMap<>();
    for (Alliance a : Alliance.values()) {
      List<Map<String, Object>> list = new ArrayList<>();
      for (HubShotSpot s : HubShotSpot.values()) {
        if (s.isFixed()) {
          Pose2d p = HubTargets.shootingPose(layout, a, s.angleDeg);
          list.add(Map.of("label", s.label, "pose", pose(p)));
        }
      }
      spots.put(a.toString(), list);
    }
    f.put("spots", spots);
    return f;
  }

  private static double[] pose(Pose2d p) {
    return new double[] {r(p.getX()), r(p.getY()), r(p.getRotation().getRadians())};
  }

  private static double r(double v) {
    return Math.round(v * 1000.0) / 1000.0;
  }
}
