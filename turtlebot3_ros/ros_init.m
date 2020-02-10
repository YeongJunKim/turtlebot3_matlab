function ros_init(ip)
    rosshutdown;
    if(ip == 'localhost')
    rosinit();
    else
    rosinit(ip);
    end
end