from dataclasses import dataclass

@dataclass
class R_vec:
    pitch: float = 0
    roll: float = 0
    yaw: float = 0
    
    def avrg(vec_list):
        """avrg a list of R_vecs

        Args:
            vec_list (list[R_vec]): list of R_vec

        Returns:
            R_vec: avraged Rvec
        """
        dev = len(vec_list)
        p, r, y = 0,0,0
        for i in vec_list:
            p += i.pitch
            r += i.roll
            y += i.yaw
        return R_vec(p/dev, r/dev, y/dev)



@dataclass()
class T_vec:
    """
    a simple dataclass to handle t vecs
    """
    x: float = 0
    y: float = 0
    z: float = 0

    def round_self(self, round_num):
        self.x = round(self.x, round_num)
        self.y = round(self.y, round_num)
        self.z = round(self.z, round_num)

    @staticmethod
    def avrg_list(vec_list):
        """
        avrg a list of T_vecs

        Args:
            vec_list (list[T_vecs]): list of tvecs

        Returns:
            T_vec: avraged Tvec
        """
        dev = len(vec_list)
        x, y, z = 0,0,0
        for i in vec_list:
            x += i.x
            y += i.y
            z += i.z
        return T_vec(x/dev, y/dev, z/dev)

class navigation_marker():
    
    def __init__(self, id, size, abs_x = 0.0, abs_y = 0.0, abs_z = 0.0, abs_yaw = 0.0) -> None:
        self.ID = id
        self.SIZE = size
        
        self.A_T = T_vec(abs_x,abs_y,abs_z)
        self.YAW = abs_yaw
        
    def get_abs_location(self, T, yaw):
        """
        given a vec from this marker returns abs pos

        Args:
            T (T_vec): 3d T vector
            yaw (float): yaw

        Returns:
            tuple(T_vec, float): t vec and yaw from origin
        """
        if self.YAW == 0:
            return T_vec(
                self.A_T.x + T.x,
                self.A_T.y - T.y, # you get positive dist from aruco y
                self.A_T.z + T.z
            ), (self.YAW + yaw)
        
        if self.YAW == 90:
            return T_vec(
                self.A_T.x + T.y,
                self.A_T.y + T.x,
                self.A_T.z + T.z
            ), (self.YAW + yaw)
            
        if self.YAW == 180:
            return T_vec(
                self.A_T.x - T.x,
                self.A_T.y + T.y,
                self.A_T.z + T.z
            ), (self.YAW + yaw)

        if self.YAW == 270:
            return T_vec(
                self.A_T.x - T.y,
                self.A_T.y - T.x,
                self.A_T.z + T.z
            ), (self.YAW + yaw)
