import processing.opengl.*;
import peasy.*;

// Modos de contacto entre dedo y objeto
enum ContactMode { FIXED, ROLLING, SLIDING }

// Parámetros geométricos y físicos de los dedos y objeto
float link_1 = 30, link_2 = 20, link_3 = 15;           // Longitudes de los segmentos del dedo
float fingertip_radius = 3;                              // Radio del extremo del dedo
float collision_avoidance_gain = 100.0;                  // Factor de repulsión para evitar colisiones

// Variables para el dedo 1
PVector finger_base_1;
float[] finger_angles_1 = new float[3];
ContactMode current_mode_1 = ContactMode.FIXED;
PVector prev_prism_contact_1 = null;
PVector prev_finger_contact_1 = null;

// Variables para el dedo 2
PVector finger_base_2;
float[] finger_angles_2 = new float[3];
ContactMode current_mode_2 = ContactMode.FIXED;
PVector prev_prism_contact_2 = null;
PVector prev_finger_contact_2 = null;

// Variables para el dedo 3
PVector finger_base_3;
float[] finger_angles_3 = new float[3];
ContactMode current_mode_3 = ContactMode.FIXED;
PVector prev_prism_contact_3 = null;
PVector prev_finger_contact_3 = null;

// Parámetros y estado del objeto (prisma)
PVector prism_pos;
PVector prism_rot;
float prism_w = 5, prism_h = 40, prism_d = 60;
PMatrix3D prism_transform = new PMatrix3D();

float prism_mass = 1.0;
float gravity_accel = 9.81;
float stiffness_coeff = 1.2;
float friction_coefficient = 2.5;
boolean stable_grasp = false;

// Fuerzas y puntos de contacto en cada dedo
PVector force_finger_1 = new PVector();
PVector force_finger_2 = new PVector();
PVector force_finger_3 = new PVector();
PVector contact_pt_finger_1 = new PVector();
PVector contact_pt_finger_2 = new PVector();
PVector contact_pt_finger_3 = new PVector();

// Fuerza y torque netos en el objeto
PVector net_force_global = new PVector();
PVector net_torque_global = new PVector();
float torque_threshold = 0.5;

// Velocidad del objeto y frecuencia de simulación
PVector prism_vel = new PVector(0, 0, 0);
int simulation_steps_per_second = 60;

// Cámara para navegación 3D
PeasyCam cam;

// Clase que define la configuración (posición y rotación) de un objetivo
class Obj_Config {
  PVector pos;
  PVector rot;
  Obj_Config(PVector p, PVector r) { 
    pos = p; 
    rot = r; 
  }
}

// Lista de configuraciones objetivo
ArrayList<Obj_Config> qf_list = new ArrayList<Obj_Config>();
int current_target_index = 0;
float target_threshold = 5.0;

void setup() {
  size(800, 600, P3D);
  noStroke();
  lights();
  cam = new PeasyCam(this, 500);
  
  // Inicialización de bases y ángulos de los dedos
  finger_base_1 = new PVector(0, 0, 0);
  finger_base_2 = new PVector(-40, 0, -15);
  finger_base_3 = new PVector(-40, 0, 15);
  for (int i = 0; i < 3; i++) {
    finger_angles_1[i] = 0;
    finger_angles_2[i] = 0;
    finger_angles_3[i] = 0;
  }
  
  // Posición y rotación inicial del objeto
  prism_pos = new PVector(-20, 60, 0);
  prism_rot = new PVector(0, 0, 0);
  updatePrismTransform();
  
  // Inicialización de configuraciones objetivo (posición y rotación)
  qf_list.add(new Obj_Config(new PVector(-10, 60, 0), new PVector(0, 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-40, 60, 0), new PVector(0, 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 0), new PVector(0, 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 50, 0), new PVector(0, 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 70, 0), new PVector(0, 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 10), new PVector(0, 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 60, -10), new PVector(0, 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 0), new PVector(0, 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 0), new PVector(radians(30), 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 0), new PVector(radians(-30), 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 0), new PVector(0, 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 0), new PVector(0, radians(30), 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 0), new PVector(0, radians(-30), 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 0), new PVector(0, 0, 0)));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 0), new PVector(0, 0, radians(30))));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 0), new PVector(0, 0, radians(-30))));
  qf_list.add(new Obj_Config(new PVector(-20, 60, 0), new PVector(0, 0, 0)));
}

void draw() {
  background(255);               // Fondo blanco
  lights();

  // Actualización de cinemática inversa para cada dedo
  doInverseKinematicsForFinger(1);
  doInverseKinematicsForFinger(2);
  doInverseKinematicsForFinger(3);
  
  // Cálculo de estabilidad del agarre y dinámica del objeto
  computeGraspStability();
  applyPhysicsToPrism();
  updateTargetConfiguration();
  
  // Traslación para ajustar la visualización
  translate(0, 30, 0);
  
  // Dibujo de dedos y objeto
  drawFinger(1, force_finger_1);
  drawFinger(2, force_finger_2);
  drawFinger(3, force_finger_3);
  drawPrism();
  
  // Visualización de estado de contacto y estabilidad del agarre
  hint(DISABLE_DEPTH_TEST);
  fill(0);
  textSize(16);
  text("Dedo#1: " + current_mode_1 + " | Dedo#2: " + current_mode_2 + " | Dedo#3: " + current_mode_3, 10, height - 10);
  text(stable_grasp ? "Grasp: STABLE" : "Grasp: UNSTABLE", 10, height - 30);
  hint(ENABLE_DEPTH_TEST);
}

// Calcula la posición de cada articulación del dedo (cinemática directa)
PVector[] forwardKinematics(PVector base, float[] angles) {
  PVector[] out = new PVector[4];
  out[0] = base.copy();
  float t1 = angles[0], t2 = angles[1], t3 = angles[2];
  float x1 = 0, y1 = link_1 * cos(t1), z1 = link_1 * sin(t1);
  out[1] = PVector.add(out[0], new PVector(x1, y1, z1));
  float x2 = -link_2 * sin(t2), y2 = link_2 * cos(t2);
  float y2r = y2 * cos(t1), z2r = y2 * sin(t1);
  out[2] = PVector.add(out[1], new PVector(x2, y2r, z2r));
  float x3 = -link_3 * sin(t2 + t3), y3 = link_3 * cos(t2 + t3);
  float y3r = y3 * cos(t1), z3r = y3 * sin(t1);
  out[3] = PVector.add(out[2], new PVector(x3, y3r, z3r));
  return out;
}

// Ejecuta la cinemática inversa para un dedo específico
void doInverseKinematicsForFinger(int finger_id) {
  float[] angles;
  PVector base;
  ContactMode current_mode;
  PVector prev_prism_contact;
  PVector prev_finger_contact;
  
  // Selección de parámetros según el dedo
  if (finger_id == 1) {
    angles = finger_angles_1;
    base = finger_base_1;
    current_mode = current_mode_1;
    prev_prism_contact = prev_prism_contact_1;
    prev_finger_contact = prev_finger_contact_1;
  } else if (finger_id == 2) {
    angles = finger_angles_2;
    base = finger_base_2;
    current_mode = current_mode_2;
    prev_prism_contact = prev_prism_contact_2;
    prev_finger_contact = prev_finger_contact_2;
  } else {
    angles = finger_angles_3;
    base = finger_base_3;
    current_mode = current_mode_3;
    prev_prism_contact = prev_prism_contact_3;
    prev_finger_contact = prev_finger_contact_3;
  }
  
  // Almacena la postura previa para suavizado en el dedo 1
  float[] prev_angles = new float[3];
  if (finger_id == 1) {
    for (int i = 0; i < 3; i++) {
      prev_angles[i] = angles[i];
    }
  }
  
  PVector[] joints_pos = forwardKinematics(base, angles);
  PVector tip_center = joints_pos[3];
  PVector contact_on_prism = getClosestPointOnPrism(tip_center);
  PVector dir = PVector.sub(contact_on_prism, tip_center);
  if (dir.mag() < 1e-5) dir.set(0, 1, 0);
  else dir.normalize();
  PVector target = PVector.sub(contact_on_prism, PVector.mult(dir, fingertip_radius));
  
  int iterations = 5;
  float damping = 0.3;
  for (int i = 0; i < iterations; i++) {
    joints_pos = forwardKinematics(base, angles);
    tip_center = joints_pos[3];
    PVector error = PVector.sub(target, tip_center);
    PVector repulsion = computeFingerRepulsionError(angles, base, finger_id);
    error.add(repulsion);
    if (error.mag() < 0.01) break;
    float[][] J = computeJacobian(base, angles);
    float[][] invJ = invertMatrix3x3(J);
    float[] delta_ang = new float[3];
    for (int k = 0; k < 3; k++) {
      delta_ang[k] = invJ[k][0] * error.x + invJ[k][1] * error.y + invJ[k][2] * error.z;
      angles[k] += damping * delta_ang[k];
    }
  }
  validateAngles(angles);
  
  // Suaviza la solución de IK interpolando con la postura anterior (aplica solo al dedo 1)
  if (finger_id == 1) {
    for (int i = 0; i < 3; i++) {
      angles[i] = lerp(prev_angles[i], angles[i], 0.5);
    }
  }
  
  // Detección y aplicación de restricciones según el modo de movimiento
  ContactMode new_mode = detectMovementMode(contact_on_prism, tip_center, current_mode,
                                              prev_prism_contact, prev_finger_contact);
  current_mode = new_mode;
  if (current_mode == ContactMode.ROLLING) {
    applyRollingConstraint(contact_on_prism, tip_center, angles, prev_prism_contact, prev_finger_contact);
  } else if (current_mode == ContactMode.SLIDING) {
    applySlidingConstraint(contact_on_prism, tip_center, angles, prev_finger_contact);
  }
  fingerRelocationStrategy(base, angles, tip_center, contact_on_prism);
  
  // Actualización de variables según el dedo
  if (finger_id == 1) {
    finger_angles_1 = angles;
    current_mode_1 = current_mode;
    prev_prism_contact_1 = contact_on_prism.copy();
    prev_finger_contact_1 = tip_center.copy();
  } else if (finger_id == 2) {
    finger_angles_2 = angles;
    current_mode_2 = current_mode;
    prev_prism_contact_2 = contact_on_prism.copy();
    prev_finger_contact_2 = tip_center.copy();
  } else {
    finger_angles_3 = angles;
    current_mode_3 = current_mode;
    prev_prism_contact_3 = contact_on_prism.copy();
    prev_finger_contact_3 = tip_center.copy();
  }
}

// Calcula la matriz Jacobiana para la cinemática inversa
float[][] computeJacobian(PVector base, float[] angles) {
  PVector tip = forwardKinematics(base, angles)[3];
  float epsilon = 0.001;
  float[][] J = new float[3][3];
  for (int i = 0; i < 3; i++) {
    float old_angle = angles[i];
    angles[i] = old_angle + epsilon;
    PVector tip_perturbed = forwardKinematics(base, angles)[3];
    angles[i] = old_angle;
    PVector diff = PVector.sub(tip_perturbed, tip);
    J[0][i] = diff.x / epsilon;
    J[1][i] = diff.y / epsilon;
    J[2][i] = diff.z / epsilon;
  }
  return J;
}

// Invierte una matriz 3x3
float[][] invertMatrix3x3(float[][] M) {
  float det = M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
            - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
            + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
  if (abs(det) < 1e-6) return new float[][]{{1,0,0},{0,1,0},{0,0,1}};
  float invDet = 1.0 / det;
  float[][] inv = new float[3][3];
  inv[0][0] =  (M[1][1]*M[2][2]-M[1][2]*M[2][1]) * invDet;
  inv[0][1] = -(M[0][1]*M[2][2]-M[0][2]*M[2][1]) * invDet;
  inv[0][2] =  (M[0][1]*M[1][2]-M[0][2]*M[1][1]) * invDet;
  inv[1][0] = -(M[1][0]*M[2][2]-M[1][2]*M[2][0]) * invDet;
  inv[1][1] =  (M[0][0]*M[2][2]-M[0][2]*M[2][0]) * invDet;
  inv[1][2] = -(M[0][0]*M[1][2]-M[0][2]*M[1][0]) * invDet;
  inv[2][0] =  (M[1][0]*M[2][1]-M[1][1]*M[2][0]) * invDet;
  inv[2][1] = -(M[0][0]*M[2][1]-M[0][1]*M[2][0]) * invDet;
  inv[2][2] =  (M[0][0]*M[1][1]-M[0][1]*M[1][0]) * invDet;
  return inv;
}

// Limita los ángulos dentro de un rango permitido
void validateAngles(float[] angles) {
  for (int i = 0; i < 3; i++) {
    angles[i] = constrain(angles[i], radians(-90), radians(90));
  }
}

// Calcula un término de repulsión para evitar intersección del dedo con el objeto
PVector computeFingerRepulsionError(float[] angles, PVector base, int finger_id) {
  PVector repulsion = new PVector(0, 0, 0);
  PVector[] joints_pos = forwardKinematics(base, angles);
  int samples_per_segment = 10;
  int count = 0;
  for (int i = 0; i < joints_pos.length - 1; i++) {
    for (int j = 0; j <= samples_per_segment; j++) {
      float t = j / (float) samples_per_segment;
      PVector sample = PVector.lerp(joints_pos[i], joints_pos[i+1], t);
      if (isPointInsidePrism(sample)) {
        PVector closest = getClosestPointOnPrism(sample);
        PVector diff = PVector.sub(sample, closest);
        if (diff.mag() < 1e-5) diff.set(0, 1, 0);
        else diff.normalize();
        diff.mult(collision_avoidance_gain);
        repulsion.add(diff);
        count++;
      }
    }
  }
  if (count > 0) repulsion.div(count);
  return repulsion;
}

// Determina el modo de contacto basándose en la variación de posiciones
ContactMode detectMovementMode(PVector prism_c, PVector finger_c,
                               ContactMode curr_mode,
                               PVector prev_prism, PVector prev_finger) {
  if (prev_prism == null || prev_finger == null) return ContactMode.FIXED;
  float dist_finger = PVector.dist(finger_c, prev_finger);
  float dist_prism  = PVector.dist(prism_c, prev_prism);
  return (dist_finger < 0.1 && dist_prism < 0.1) ? ContactMode.FIXED 
       : (abs(dist_finger - dist_prism) < 0.2 ? ContactMode.ROLLING : ContactMode.SLIDING);
}

// Ajusta el ángulo en modo ROLLING para compensar diferencias en desplazamientos
void applyRollingConstraint(PVector prism_c, PVector finger_c,
                            float[] angles,
                            PVector prev_prism, PVector prev_finger) {
  if (prev_prism == null || prev_finger == null) return;
  float dist_finger = PVector.dist(finger_c, prev_finger);
  float dist_prism  = PVector.dist(prism_c, prev_prism);
  float diff = dist_prism - dist_finger;
  if (abs(diff) > 0.01) {
    float adjustment = friction_coefficient * 0.002 * diff;
    angles[2] += adjustment;
  }
}

// Ajusta el ángulo en modo SLIDING cuando se excede un desplazamiento umbral
void applySlidingConstraint(PVector prism_c, PVector finger_c,
                            float[] angles,
                            PVector prev_finger) {
  if (prev_finger == null) return;
  float dist_finger = PVector.dist(finger_c, prev_finger);
  if (dist_finger > 0.2) {
    float adjustment = friction_coefficient * 0.005 * dist_finger;
    angles[2] -= adjustment;
  }
}

// Estrategia de recolocación del dedo si se aleja demasiado del objeto
void fingerRelocationStrategy(PVector base, float[] angles,
                              PVector tip_center, PVector contact) {
  float dist_to_prism = PVector.dist(tip_center, contact);
  if (dist_to_prism > 60) {
    for (int i = 0; i < 3; i++) {
      angles[i] = lerp(angles[i], 0, 0.1);
    }
  }
}

// Retorna el punto más cercano sobre el objeto a una posición dada
PVector getClosestPointOnPrism(PVector p_world) {
  PMatrix3D inv = prism_transform.get();
  inv.invert();
  PVector local_p = new PVector();
  inv.mult(p_world, local_p);
  float hx = prism_w / 2, hy = prism_h / 2, hz = prism_d / 2;
  local_p.x = constrain(local_p.x, -hx, hx);
  local_p.y = constrain(local_p.y, -hy, hy);
  local_p.z = constrain(local_p.z, -hz, hz);
  PVector w = new PVector();
  prism_transform.mult(local_p, w);
  return w;
}

// Actualiza la transformación del objeto según su posición y rotación
void updatePrismTransform() {
  prism_transform.reset();
  prism_transform.translate(prism_pos.x, prism_pos.y, prism_pos.z);
  prism_transform.rotateX(prism_rot.x);
  prism_transform.rotateY(prism_rot.y);
  prism_transform.rotateZ(prism_rot.z);
}

// Verifica si un punto se encuentra dentro de los límites del objeto
boolean isPointInsidePrism(PVector p) {
  PMatrix3D inv = prism_transform.get();
  inv.invert();
  PVector local_p = new PVector();
  inv.mult(p, local_p);
  float hx = prism_w / 2, hy = prism_h / 2, hz = prism_d / 2;
  return (local_p.x >= -hx && local_p.x <= hx &&
          local_p.y >= -hy && local_p.y <= hy &&
          local_p.z >= -hz && local_p.z <= hz);
}

// Dibuja un cilindro entre dos puntos para representar segmentos del dedo
void drawCylinderBetweenPoints(PVector p1, PVector p2, float r) {
  PVector dir = PVector.sub(p2, p1);
  float h = dir.mag();
  pushMatrix();
  translate(p1.x, p1.y, p1.z);
  PVector yAxis = new PVector(0, 1, 0);
  PVector axis = yAxis.cross(dir);
  float angle = acos(yAxis.dot(dir) / h);
  if (axis.mag() > 0.0001) {
    axis.normalize();
    rotate(angle, axis.x, axis.y, axis.z);
  }
  cylinder(r, h);
  popMatrix();
}

// Dibuja un cilindro con base circular
void cylinder(float r, float h) {
  int sides = 20;
  float angleStep = TWO_PI / sides;
  beginShape(QUAD_STRIP);
  for (int i = 0; i <= sides; i++) {
    float angle = i * angleStep;
    float x = cos(angle) * r;
    float z = sin(angle) * r;
    vertex(x, 0, z);
    vertex(x, h, z);
  }
  endShape();
  beginShape(TRIANGLE_FAN);
  vertex(0, 0, 0);
  for (int i = 0; i <= sides; i++) {
    float angle = i * angleStep;
    float x = cos(angle) * r;
    float z = sin(angle) * r;
    vertex(x, 0, z);
  }
  endShape();
  beginShape(TRIANGLE_FAN);
  vertex(0, h, 0);
  for (int i = 0; i <= sides; i++) {
    float angle = i * angleStep;
    float x = cos(angle) * r;
    float z = sin(angle) * r;
    vertex(x, h, z);
  }
  endShape();
}

// Dibuja el dedo especificado y representa la fuerza de contacto si es significativa
void drawFinger(int finger_id, PVector force_vec) {
  float[] angles;
  PVector base;
  if (finger_id == 1) {
    angles = finger_angles_1;
    base   = finger_base_1;
  } else if (finger_id == 2) {
    angles = finger_angles_2;
    base   = finger_base_2;
  } else {
    angles = finger_angles_3;
    base   = finger_base_3;
  }
  PVector[] pos = forwardKinematics(base, angles);
  pushMatrix();
  translate(pos[0].x, pos[0].y, pos[0].z);
  fill(150);
  box(10);
  popMatrix();
  stroke(200);
  for (int i = 0; i < 3; i++) {
    drawCylinderBetweenPoints(pos[i], pos[i+1], 3);
  }
  noStroke();
  for (int i = 0; i < pos.length; i++) {
    pushMatrix();
    translate(pos[i].x, pos[i].y, pos[i].z);
    fill(100, 150, 255);
    sphere(4);
    popMatrix();
  }
  pushMatrix();
  translate(pos[3].x, pos[3].y, pos[3].z);
  fill(0, 100, 255);
  sphere(fingertip_radius);
  popMatrix();
  if (force_vec.mag() > 0.01) drawForceLine(pos[3], force_vec);
}

// Dibuja el objeto (prisma)
void drawPrism() {
  pushMatrix();
  applyMatrix(prism_transform);
  fill(200, 0, 0);
  box(prism_w, prism_h, prism_d);
  popMatrix();
}

// Representa gráficamente la línea de fuerza aplicada en el dedo
void drawForceLine(PVector origin, PVector force) {
  float scale = 20.0;
  PVector tip = PVector.add(origin, PVector.mult(force, scale));
  strokeWeight(3);
  stroke(0, 255, 0);
  line(origin.x, origin.y, origin.z, tip.x, tip.y, tip.z);
  noStroke();
}

// Calcula la estabilidad del agarre combinando fuerzas y torques de los dedos
void computeGraspStability() {
  force_finger_1 = computeFingerContactForce(finger_base_1, finger_angles_1, contact_pt_finger_1);
  force_finger_2 = computeFingerContactForce(finger_base_2, finger_angles_2, contact_pt_finger_2);
  force_finger_3 = computeFingerContactForce(finger_base_3, finger_angles_3, contact_pt_finger_3);
  PVector total_force = PVector.add(force_finger_1, force_finger_2);
  total_force.add(force_finger_3);
  PVector torque1 = computeTorqueOnPrism(force_finger_1, contact_pt_finger_1);
  PVector torque2 = computeTorqueOnPrism(force_finger_2, contact_pt_finger_2);
  PVector torque3 = computeTorqueOnPrism(force_finger_3, contact_pt_finger_3);
  PVector net_torque = PVector.add(torque1, torque2);
  net_torque.add(torque3);
  PVector weight_vec = new PVector(prism_mass * gravity_accel, 0, 0);
  PVector net_force = PVector.add(total_force, weight_vec);
  net_force_global.set(net_force);
  net_torque_global.set(net_torque);
  boolean force_balance  = (net_force.mag() < 100.0 || net_force.z > 0);
  boolean torque_balance = (net_torque.mag() < torque_threshold);
  stable_grasp = force_balance && torque_balance;
}

// Calcula la fuerza de contacto en el dedo basada en la penetración y fricción
PVector computeFingerContactForce(PVector base, float[] angles, PVector contact_point_out) {
  PVector[] joints_pos = forwardKinematics(base, angles);
  PVector tip_center = joints_pos[3];
  PVector contact_on_prism = getClosestPointOnPrism(tip_center);
  contact_point_out.set(contact_on_prism);
  float dist = PVector.dist(tip_center, contact_on_prism);
  float penetration = fingertip_radius - dist;
  if (penetration < 0) return new PVector(0,0,0);
  float normal_mag = stiffness_coeff * penetration;
  PVector normal_dir = PVector.sub(tip_center, contact_on_prism);
  normal_dir.normalize();
  PVector normal_force = PVector.mult(normal_dir, normal_mag);
  float max_friction = friction_coefficient * normal_mag;
  float desired_tangential = 0.1;
  PVector friction_force;
  if (desired_tangential <= max_friction)
    friction_force = PVector.mult(new PVector(1, 0, 0), -desired_tangential);
  else
    friction_force = PVector.mult(new PVector(1, 0, 0), -max_friction);
  return PVector.add(normal_force, friction_force);
}

// Calcula el torque generado en el objeto a partir de una fuerza y su punto de aplicación
PVector computeTorqueOnPrism(PVector force, PVector contact_point) {
  PVector r = PVector.sub(contact_point, prism_pos);
  return r.cross(force);
}

// Aplica la dinámica al objeto según las fuerzas netas y estabilidad del agarre
void applyPhysicsToPrism() {
  float dt = 1.0 / 60.0;
  PVector accel = PVector.mult(net_force_global, 1.0 / prism_mass);
  if (!stable_grasp) {
    prism_vel.add(PVector.mult(accel, dt));
    prism_pos.add(PVector.mult(prism_vel, dt));
    prism_vel.mult(0.98);
  } else {
    prism_vel.set(0, 0, 0);
  }
  updatePrismTransform();
}

// Actualiza la configuración objetivo y realiza la interpolación del objeto
void updateTargetConfiguration() {
  if (current_target_index < qf_list.size()) {
    Obj_Config target_config = qf_list.get(current_target_index);
    float pos_diff = PVector.dist(prism_pos, target_config.pos);
    float rot_diff = PVector.dist(prism_rot, target_config.rot);
    if (pos_diff < target_threshold && rot_diff < 0.1) {
      println("Meta alcanzada: " + current_target_index);
      current_target_index++;
    } else {
      if (stable_grasp) {
        float alpha = 0.02;
        prism_pos = PVector.lerp(prism_pos, target_config.pos, alpha);
        prism_rot = PVector.lerp(prism_rot, target_config.rot, alpha);
        updatePrismTransform();
      }
    }
  }
}
