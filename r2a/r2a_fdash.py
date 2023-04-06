from r2a.ir2a import IR2A
from player.parser import *
import time
from statistics import mean
import numpy as np


class R2A_FDash(IR2A):

    def __init__(self, id):
        IR2A.__init__(self, id)
        self.throughputs = []
        self.request_time = 0
        self.qi = []
        # t[i] e t[i-1]
        self.buffers = [-1, -1]
        self.target_buffer = 15

    def open_left(self, x, alpha, beta):
        if x <= alpha:
            return 1
        if alpha < x and x < beta:
            return (beta - x) / (beta - alpha)
        else:
            return 0

    def open_right(self, x, alpha, beta):
        if x <= alpha:
            return 0
        if alpha < x and x < beta:
            return (x - alpha) / (beta - alpha)
        else:
            return 1

    def triangular(self, x, a, b, c):
        return max(min((x - a) / (b - a), (c - x) / (c - b)), 0)

    # partições da figura 2a
    def partition_buffering_time(self, t):
        # valor da tabela 1
        T = self.target_buffer
        SH = 0
        C = 0
        L = 0

        if 0 <= t < T:
            SH = self.open_left(t, 2 * T / 3, T)
        if 2 * T / 3 < t < 4 * T:
            C = self.triangular(t, 2 * T / 3, T, 4 * T)
        if t > T:
            L = self.open_right(t, T, 4 * T)

        return SH, C, L

    # partições da figura 2b
    def partition_differential_of_buffering_time(self, dt):
        # valor da tabela 1
        T = self.target_buffer
        F = 0
        ST = 0
        R = 0

        if dt < 0:
            F = self.open_left(dt, -2 * T / 3, 0)
        if (-2 * T / 3) < dt < (4 * T):
            ST = self.triangular(dt, -2 * T / 3, 0, 4 * T)
        if dt > 0:
            R = self.open_right(dt, 0, 4 * T)

        return F, ST, R

    def output(self, t, dt):
        SH, C, L = self.partition_buffering_time(t)
        F, ST, R = self.partition_differential_of_buffering_time(dt)

        r1 = min(SH, F)
        r2 = min(C, F)
        r3 = min(L, F)
        r4 = min(SH, ST)
        r5 = min(C, ST)
        r6 = min(L, ST)
        r7 = min(SH, R)
        r8 = min(C, R)
        r9 = min(L, R)

        # equações (2...6)
        I = abs(r9)
        SI = np.linalg.norm(np.array((r6, r8)))
        NC = np.linalg.norm(np.array((r3, r5, r7)))
        SR = np.linalg.norm(np.array((r2, r4)))
        R = abs(r1)

        # valores da tabela 1
        # equação (1)
        f = (0.25 * R + 0.5 * SR + 1 * NC + 1.5 * SI + 2 * I) / (SR + R + NC + SI + I)
        return f

    def handle_xml_request(self, msg):
        self.request_time = time.perf_counter()
        self.send_down(msg)

    def handle_xml_response(self, msg):

        parsed_mpd = parse_mpd(msg.get_payload())
        self.qi = parsed_mpd.get_qi()

        t = time.perf_counter() - self.request_time
        self.throughputs.append(msg.get_bit_length() / t)

        self.send_up(msg)

    def handle_segment_size_request(self, msg):

        self.request_time = time.perf_counter()

        print(self.buffers)
        # inicio ele como 1 pra não bugar até ter o t[i] e t[i-1]
        f = 1

        # gambiarra pra ele só fazer a conta quanto tiver o  t[i] e t[i-1]
        if self.buffers[1] > -1:
            buffer = self.buffers[0]
            last_buffer = self.buffers[1]
            # t e Δt
            f = self.output(buffer, buffer - last_buffer)

        # equação (9)
        avg = mean(self.throughputs[-10:])
        # equação (8)
        b = avg * f

        selected_qi = self.qi[0]
        for i in self.qi:
            if b > i:
                selected_qi = i

        msg.add_quality_id(selected_qi)
        self.send_down(msg)

    def handle_segment_size_response(self, msg):
        t = time.perf_counter() - self.request_time
        lista_buffers = self.whiteboard.get_playback_buffer_size()

        # dois ultimos valores do buffer no momomento que um pacote é recebido
        # t[i] e t[i-1]
        if len(lista_buffers) > 1:
            self.buffers.insert(0, lista_buffers[-1][1])
            self.buffers = self.buffers[:-1]

        # equação (7)
        r = msg.get_bit_length() / t
        self.throughputs.append(r)
        self.send_up(msg)

    def initialize(self):
        pass

    def finalization(self):
        pass
