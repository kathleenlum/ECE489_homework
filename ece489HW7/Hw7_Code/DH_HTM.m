function T = DH_HTM(a, al, d, t, i)
        T = [cos(t(i)), -sin(t(i))*cos(al(i)), sin(t(i))*sin(al(i)), a(i)*cos(t(i));
             sin(t(i)), cos(t(i))*cos(al(i)), -cos(t(i))*sin(al(i)), a(i)*sin(t(i));
             0, sin(al(i)), cos(al(i)), d(i);
             0, 0, 0, 1];

end
