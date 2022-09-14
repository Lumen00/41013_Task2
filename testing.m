close all;
d = Dobot(true);
d.model.base = d.model.base*trotx(pi/2);
d.model.animate(zeros(1,6));
d.model.teach;