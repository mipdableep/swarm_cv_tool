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
    
    def subtract(vec1, vec2):
        """vec1 - vec2

        Args:
            vec1 (R_vec): subtructee
            vec2 (R_vec): subtructor

        Returns:
            R_vec: vec1 - vec2
        """
        return R_vec(
            vec1.pitch - vec2.pitch,
            vec1.roll - vec2.roll,
            vec1.yaw - vec2.yaw,
        )

@dataclass
class T_vec:
    lr: float = 0
    fb: float = 0
    ud: float = 0

    @staticmethod
    def avrg(vec_list):
        """avrg a list of T_vecs

        Args:
            vec_list (list[tvecs]): list of tvecs

        Returns:
            T_vec: avraged Tvec
        """
        dev = len(vec_list)
        x, y, z = 0,0,0
        for i in vec_list:
            x += i.lr
            y += i.fb
            z += i.ud
        return T_vec(x/dev, y/dev, z/dev)
    
    @staticmethod
    def subtract(vec1, vec2):
        """vec1 - vec2

        Args:
            vec1 (T_vec): subtructee
            vec2 (T_vec): subtructor

        Returns:
            T_vec: vec1 - vec2
        """
        return T_vec(
            vec1.lr - vec2.lr,
            vec1.fb - vec2.fb,
            vec1.ud - vec2.ud,
        )

class navigation_marker():
    def __init__(self, id, size, abs_x = 0, abs_y = 0, abs_z = 0, abs_pitch = 0, abs_roll = 0, abs_yaw = 0) -> None:
        self.ID = id
        self.SIZE = size
        
        self.A_T = T_vec(abs_x,abs_y,abs_z)
        self.A_R = R_vec(abs_pitch, abs_roll, abs_yaw)