clc
sim('OMNI_BOT_MOEDL2');
x1 = out.x_set.Data(:,1);
y1 = out.y_set.Data(:,1);
x2 = out.x_rb.Data(:,1);
y2 = out.y_rb.Data(:,1);
plot(x1,y1,x2,y2);