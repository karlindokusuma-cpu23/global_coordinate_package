import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class GlobalCoordinateNode(Node):
    def __init__(self):
        super().__init__("global_coordinate_node")

        # ==========================================
        # 1. INISIALISASI POSISI DAN ORIENTASI ROBOT
        # ==========================================

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ==========================================
        # 2. VARIABEL KECEPATAN ROBOT
        # ==========================================

        self.v_linear_x = 0.0
        self.v_linear_y = 0.0
        self.omega = 0.0

        # Kecepatan dalam koordinat global
        self.vx_global = 0.0
        self.vy_global = 0.0

        # Waktu awal
        self.start_time = self.get_clock().now()
        self.last_time = self.start_time

        # Untuk membatasi frekuensi tampilan terminal
        self.last_display_time = 0.0
        self.display_interval = 0.5

        # ==========================================
        # 3. SUBSCRIBER CMD_VEL
        # ==========================================

        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        # ==========================================
        # 4. TIMER PERHITUNGAN
        # ==========================================

        self.timer = self.create_timer(0.05, self.calculate_global_coordinate)

        self.get_logger().info("Global Coordinate Node berhasil dijalankan!")

    # ==============================================
    # CALLBACK: MEMBACA PERINTAH KECEPATAN
    # ==============================================

    def cmd_vel_callback(self, msg):

        self.v_linear_x = msg.linear.x
        self.v_linear_y = msg.linear.y
        self.omega = msg.angular.z

    # ==============================================
    # PERHITUNGAN KOORDINAT GLOBAL
    # ==============================================

    def calculate_global_coordinate(self):

        current_time = self.get_clock().now()

        # Selang waktu dalam detik
        dt = (current_time - self.last_time).nanoseconds / 1e9

        self.last_time = current_time

        # Hindari perhitungan jika waktu tidak valid
        if dt <= 0.0:
            return

        # Batasi dt untuk mencegah lonjakan besar
        # jika terjadi jeda pada proses.
        # Jeda besar tetap perlu diperiksa karena
        # estimasi berbasis kecepatan tidak akurat
        # ketika ada waktu yang terlewat.
        if dt > 0.2:
            dt = 0.2

        # Simpan orientasi sebelum diperbarui
        theta_old = self.theta

        # Estimasi orientasi di pertengahan interval
        theta_mid = theta_old + (self.omega * dt / 2.0)

        # ==========================================
        # A. KONVERSI KECEPATAN BODY KE GLOBAL
        # ==========================================

        self.vx_global = self.v_linear_x * math.cos(
            theta_mid
        ) - self.v_linear_y * math.sin(theta_mid)

        self.vy_global = self.v_linear_x * math.sin(
            theta_mid
        ) + self.v_linear_y * math.cos(theta_mid)

        # ==========================================
        # B. INTEGRASI UNTUK MENGESTIMASI POSISI
        # ==========================================

        self.x += self.vx_global * dt
        self.y += self.vy_global * dt

        # ==========================================
        # C. PEMBARUAN ORIENTASI
        # ==========================================

        self.theta += self.omega * dt

        # Normalisasi sudut ke rentang [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # ==========================================
        # D. TAMPILKAN INFORMASI DI TERMINAL
        # ==========================================

        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time - self.last_display_time >= (self.display_interval):
            self.last_display_time = elapsed_time
            self.display_dashboard(elapsed_time)

    # ==============================================
    # DASHBOARD TERMINAL
    # ==============================================

    def display_dashboard(self, elapsed_time):

        theta_deg = math.degrees(self.theta)

        speed_global = math.sqrt(self.vx_global**2 + self.vy_global**2)

        # Menentukan status gerakan
        if (
            abs(self.v_linear_x) < 0.001
            and abs(self.v_linear_y) < 0.001
            and abs(self.omega) < 0.001
        ):
            status = "BERHENTI"

        elif abs(self.v_linear_x) < 0.001 and abs(self.v_linear_y) < 0.001:
            status = "ROTASI DI TEMPAT"

        elif abs(self.omega) < 0.001:
            status = "BERGERAK LURUS"

        else:
            status = "BERGERAK SAMBIL BERBELOK"

        # Membersihkan tampilan lama dan kembali
        # ke posisi awal terminal
        print("\033[2J\033[H", end="")

        print("=" * 62)
        print("          ROS 2 - GLOBAL COORDINATE MONITOR")
        print("=" * 62)

        print(f" Waktu berjalan       : {elapsed_time:10.2f} s")
        print(f" Status robot         : {status}")
        print("-" * 62)

        print(" [1] PERINTAH KECEPATAN (BODY COORDINATE)")
        print(f"     Linear X         : {self.v_linear_x:10.3f} m/s")
        print(f"     Linear Y         : {self.v_linear_y:10.3f} m/s")
        print(f"     Angular Z        : {self.omega:10.3f} rad/s")

        print("-" * 62)

        print(" [2] KECEPATAN GLOBAL")
        print(f"     Vx global        : {self.vx_global:10.3f} m/s")
        print(f"     Vy global        : {self.vy_global:10.3f} m/s")
        print(f"     Resultan speed   : {speed_global:10.3f} m/s")

        print("-" * 62)

        print(" [3] ESTIMASI POSISI GLOBAL")
        print(f"     Posisi X         : {self.x:10.3f} m")
        print(f"     Posisi Y         : {self.y:10.3f} m")
        print(f"     Orientasi        : {theta_deg:10.2f} deg")
        print(f"     Orientasi        : {self.theta:10.3f} rad")

        print("-" * 62)

        print(" [4] INFORMASI")
        print("     Frame            : Global X-Y")
        print("     Sumber data      : /cmd_vel")
        print("     Metode           : Integrasi numerik")
        print("     Catatan          : Estimasi dari perintah")
        print("=" * 62)
        print(" Tekan Ctrl+C untuk menghentikan node.")
        print("=" * 62, flush=True)


def main(args=None):

    rclpy.init(args=args)

    node = GlobalCoordinateNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nGlobal Coordinate Node dihentikan.")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
