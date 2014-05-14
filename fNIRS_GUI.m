% fNIRS BCI Interface
clear all;
s1 = serial('COM29','BaudRate',9600,'DataBits',8,'Parity','None','FlowControl','None','StopBits',1);
response = 't';
while response ~= 'q'
    response = input('Type s (for start plotting data) or q (for quit)','s');
    if response == 's'
        fopen(s1);
        figure(1);
        t = 0;
        y = 0;
        h = plot(t,y,'YDataSource','y');
        while(1)
            while(s1.BytesAvailable < 4)
            end
            data = uint8(fread(s1,4));
            data2=uint16(zeros(2,1));
            data2(1)=typecast(data(1:2),'uint16');
            data2(2)=typecast(data(3:4),'uint16');
            if i==10
                y = [y data2(1)];
                t = [t (t(1)+1)];
                refreshdata(h,'caller') 
                drawnow
            end
        end
        %close serial port
        fclose(s1);
    end
end