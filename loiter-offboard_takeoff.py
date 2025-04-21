def takeoff(self, altitude: float, yaw: float = 0.0) -> bool:
        """
        1) Send TAKEOFF (22)
        2) Loiter & change altitude (186)
        3) Engage OFFBOARD mode
        """
        # 1) NAV_TAKEOFF
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param4=yaw,      # desired yaw after takeoff
            param7=altitude  # takeoff altitude
        )
        self.get_logger().info(f"Takeoff command sent: alt={altitude}m, yaw={yaw}rad")
        time.sleep(2.0)

        # 2) DO_CHANGE_ALTITUDE ⇒ LOITER at target altitude
        self.publish_vehicle_command(
            command=VEHICLE_CMD_DO_CHANGE_ALTITUDE,
            param1=altitude,  # new altitude
            param2=0.0        # frame (0 = home-relative)
        )
        self.get_logger().info(f"Loiter & change-altitude command sent: alt={altitude}m")
        time.sleep(2.0)

        # 3) Switch to OFFBOARD
        self.engage_offboard_mode()
        return True
