def takeoff(self, altitude: float, yaw: float = 0.0) -> bool:
        """
        Command the vehicle to take off.
        Uses VEHICLE_CMD_NAV_TAKEOFF (22). Altitude is relative to current home.
        """
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=0.0,       # minimum pitch (unused)
            param2=0.0,       # empty
            param3=0.0,       # empty
            param4=yaw,       # yaw angle
            param5=0.0,       # latitude (0 => current)
            param6=0.0,       # longitude
            param7=altitude   # altitude in meters
        )
        self.get_logger().info(f'Takeoff command sent: altitude={altitude}m, yaw={yaw}rad')
        time.sleep(0.5)
        return True


#uint16 VEHICLE_CMD_NAV_TAKEOFF = 22			# Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|
