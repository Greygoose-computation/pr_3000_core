config.env.g=9.81;
config.env.slope=1.0;

config.h.pivot    = 0.100;
config.h.com_load = 0.250;
config.h.com_robot = 0.300;

config.l.np1 = 0.5;
config.l.np2 = 0.4;
config.l.nr1 = 0.2;
config.l.nr2 = 0.3;

config.m.pay     = 1350;
config.m.trailer = 500;
config.m.drive = 350;
config.m.chbt = 1150;

config.frc.load_rear = 0.3;
config.frc.load_front = 0.3;
config.frc.rbt_front = 0.6;
config.frc.rbt_rear = 0.25;


save('data.mat', 'config')

