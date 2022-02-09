from tkinter import *
from tkinter import ttk

class App:
    def __init__(self, master):
        self.master = master
        self.initWidgets()

    def initWidgets(self):
       rf = ttk.Frame(self.master)
       rf.pack(side = RIGHT, fill = Y, expand = YES)
       self.lb = Listbox(rf)
       self.lb.pack(side=LEFT, fill=Y, expand=YES)
       key_values = self.show_can()
       for value in key_values.values():
           self.lb.insert(END, value)
       scroll = Scrollbar(rf, command = self.lb.yview)
       scroll.pack(side = RIGHT, fill = Y)
       self.lb['yscrollcommand'] = scroll.set
       self.lb['selectmode'] = 'extended'


       #"自动驾驶", "记录数据"
       rf1 = ttk.Frame(self.master)
       rf1.pack(side = RIGHT, fill = BOTH, expand = YES, padx = 20)
       ctrlname = ("自动驾驶", "记录数据")
       self.intVar = (IntVar(), IntVar())
       for choose in range(2):
           control = Checkbutton(rf1, text = ctrlname[choose], variable = self.intVar[choose], command = self.select,
                                 font = ('楷体', 22))
           control.grid(row = 0, column = choose)

       #"档位控制"
       Gear_text = Label(rf1, text = "档位控制", font = ('楷体', 22))
       Gear_text.grid(row = 1, column = 0)
       self.stVarG = StringVar()
       Gear_name = ("Gear_P", "Gear_R", "Gear_N", "Gear_D")
       for Gearnum in range(4):
           Gctrl = Radiobutton(rf1, text = Gear_name[Gearnum], variable = self.stVarG, command = self.change_Gear,
                               value = Gear_name[Gearnum], font = ('Times', 16))
           Gctrl.grid(row = 2, column = Gearnum)
       self.stVarG.set("Gear_P")

        #油门控制
       ACC_text = Label(rf1, text = "油门控制", font = ('楷体', 22))
       ACC_text.grid(row = 3, column = 0)
       self.intVaracc = IntVar()
       ACCenable = Checkbutton(rf1, text = "油门使能", variable = self.intVaracc, command = self.select,
                               font = ('Times', 16))
       ACCenable.grid(row = 4, column = 0)
       ACC_scale = Label(rf1, text = "油门开度", font = ('Times', 16))
       ACC_scale.grid(row = 4, column = 1)
       self.acc = ttk.Entry(rf1, width = 15, font = ('StSong', 14), foreground = 'red')
       self.acc.grid(row = 4, column = 2)
       self.acc.insert(INSERT, '0')
       ACC_range = Label(rf1, text = "(0~100)", font = ('Times', 16))
       ACC_range.grid(row = 4, column = 3)
       ACCgo = Button(rf1, text = "GO", command = self.go, font = ('Verdana', 14), width = 8)
       ACCgo.grid(row = 4, column = 4)

       #转向控制
       Steer_text = Label(rf1, text = "转向控制", font = ('楷体', 22))
       Steer_text.grid(row = 5, column = 0)
       self.intVarsteer = IntVar()
       Steerenable = Checkbutton(rf1, text = "转向使能", variable = self.intVarsteer, command = self.select,
                               font = ('Times', 16))
       Steerenable.grid(row = 6, column = 0)
       Steer_scale = Label(rf1, text = "转向角度", font = ('Times', 16))
       Steer_scale.grid(row = 6, column = 1)
       self.steer = ttk.Entry(rf1, width = 15, font = ('StSong', 14), foreground = 'red')
       self.steer.grid(row = 6, column = 2)
       self.steer.insert(INSERT, '0')
       Steer_range = Label(rf1, text = "(-570~570)", font = ('Times', 16))
       Steer_range.grid(row = 6, column = 3)
       Steergo = Button(rf1, text = "GO", command = self.go, font = ('Verdana', 14), width = 8)
       Steergo.grid(row = 6, column = 4)
       Sterate_scale = Label(rf1, text="转向速度", font=('Times', 16))
       Sterate_scale.grid(row=7, column=1)
       self.sterate = ttk.Entry(rf1, width=15, font=('StSong', 14), foreground='red')
       self.sterate.grid(row=7, column=2)
       self.sterate.insert(INSERT, '0')
       Sterate_range = Label(rf1, text="(0~1080)", font=('Times', 16))
       Sterate_range.grid(row=7, column=3)

       #制动控制
       Brake_text = Label(rf1, text = "制动控制", font = ('楷体', 22))
       Brake_text.grid(row = 8, column = 0)
       Brake_scale = Label(rf1, text = "制动减速度", font = ('Times', 16))
       Brake_scale.grid(row = 9, column = 1)
       self.brake = ttk.Entry(rf1, width = 15, font = ('StSong', 14), foreground = 'red')
       self.brake.grid(row = 9, column = 2)
       self.brake.insert(INSERT, '0')
       Brake_range = Label(rf1, text = "(0~100)", font = ('Times', 16))
       Brake_range.grid(row = 9, column = 3)
       Brakego = Button(rf1, text = "GO", command = self.go, font = ('Verdana', 14), width = 8)
       Brakego.grid(row = 9, column = 4)

       #其他控制
       Light_text = Label(rf1, text = "其他控制", font = ('楷体', 22))
       Light_text.grid(row = 10, column = 0)
       lightname = ("制动灯", "左转灯", "右转灯", "近光灯", "远光灯")
       self.intVarL = (IntVar(), IntVar(), IntVar(), IntVar(), IntVar())
       for lightnum in range(5):
            light = Checkbutton(rf1, text = lightname[lightnum], variable = self.intVarL[lightnum],
                                command = self.change_light, font=('Times', 16))
            light.grid(row = 11, column = lightnum)
       lightwe = Label(rf1, font = ('楷体', 22))
       lightwe.grid(row = 12, column = 0)

    def show_can(self):
        can_signal = dict()
        can0 = ('name', 'value')
        can1 = ('0x239', 'ESK_239')
        can2 = ('0x35f', 'PCU_0x35f')
        can3 = ('0x235', 'MCU_0x235')
        can4 = ('0x230', 'MCU_0x230')
        can_signal[can0[0]] = can0
        can_signal[can1[0]] = can1
        can_signal[can2[0]] = can2
        can_signal[can3[0]] = can3
        can_signal[can4[0]] = can4
        return can_signal


    def change_Gear(self):
        changeG = self.stVarG.get()

    def change_light(self):
        brakeL = self.intVarL[0].get()
        leftL = self.intVarL[1].get()
        rightL = self.intVarL[2].get()
        shortL = self.intVarL[3].get()
        longL = self.intVarL[4].get()
        print("brakeL:", brakeL, "leftL:", leftL, "rightL:", rightL, "shortL:",
              shortL, "longL:", longL)

    def select(self):
        mode = self.intVar[0].get()
        log = self.intVar[1].get()
        accenable = self.intVaracc.get()
        steerenable = self.intVarsteer.get()
        print("mode:", mode, "log:", log, "accenable:", accenable,
              "steerenable:", steerenable)

    def go(self):
        _acc = self.acc.get()
        _steer = self.steer.get()
        _sterate = self.sterate.get()
        _brake = self.brake.get()
        if int(_acc) >= 0 and int(_acc) <= 100:
            print(_acc)
        if int(_steer) >= -570 and int(_steer) <= 570:
            print(_steer)
        if int(_sterate) >= 0 and int(_sterate) <= 1080:
            print(_sterate)
        if int(_brake) >= 0 and int(_brake) <= 100:
            print(_brake)



root = Tk()
root.title("control_test")
App(root)
root.mainloop()

