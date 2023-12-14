Steps to make mif file:
0) made red_green file as a sanity check

1) Downloaded - rick.jpg
2) Used gimp to resize to 120*180 jpg file
3) Used matlab script to create mif file:
    src = imread('C:\Users\jamiespa\Downloads\rick2_rick120_160.jpg');
    R = src(:,:,1); 
    G = src(:,:,2); 
    B = src(:,:,3);
    imshow(src) 
    
    N = 160*120; %your ram or rom depth。
    word_len = 24; 

    fid=fopen('image.mif', 'w'); % open mif file 
    fprintf(fid, 'DEPTH=%d;\n', N);
    fprintf(fid, 'WIDTH=%d;\n', word_len);

    fprintf(fid, 'ADDRESS_RADIX = UNS;\n'); 
    fprintf(fid, 'DATA_RADIX = HEX;\n'); 
    fprintf(fid, 'CONTENT\t');
    fprintf(fid, 'BEGIN\n');
    for i = 0 : N-1
    fprintf(fid, '\t%d\t:\t%x%x%x;\n',i, R(i+1), G(i+1), B(i+1));
    end
    fprintf(fid, 'END;\n'); % prinf the end
    fclose(fid); % close your file
4) put image.mif file in the spot it needed to be