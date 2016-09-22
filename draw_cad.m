% 11 September 2015
% script to test and animate motion

filename = constants.filename;
type = constants.animation_type;
close all

% load model

load('TacSAT.mat');

TV = 1/300*[TV(:,1) TV(:,2) TV(:,3)]*ROT2(pi/2)';
boresight = constants.sen;
% set up the figure window
figure('color','w','units','normalized','outerposition',[0 0 1 1])
hold all

[sph.x, sph.y, sph.z]=sphere(100);
% radius of cone at unit length
% loop over the number of constraints and draw them all
h_cyl = zeros(constants.num_con,1);
for ii = 1:constants.num_con
    
    H = cos(constants.con_angle(ii)); %// height
    R = sin(constants.con_angle(ii)); %// chord length
    N = 100; %// number of points to define the circumference
    [cyl.x, cyl.y, cyl.z] = cylinder([0 R], N);
    cyl.z = cyl.z*H;
    % calculate the rotation matrix to go from pointing in e3 direction to
    % along the con unit vector
    if sum(constants.con(:,ii) == [0;0;1]) == 3
        dcm = eye(3,3);
    elseif sum(constants.con(:,ii) == [0;0;-1]) == 3
        dcm = ROT1(pi);
    else
        k_hat = cross([0;0;1],constants.con(:,ii));
        angle = acos(dot([0;0;1],constants.con(:,ii)));
        dcm = eye(3,3) + hat_map(k_hat) + hat_map(k_hat)*hat_map(k_hat)*(1-cos(angle))/sin(angle)^2;
    end
    
    cyl_open = dcm*[cyl.x(2,:);cyl.y(2,:);cyl.z(2,:)];
    cyl.x(2,:) = cyl_open(1,:);
    cyl.y(2,:) = cyl_open(2,:);
    cyl.z(2,:) = cyl_open(3,:);
    
    
    h_cyl(ii) = surf(cyl.x,cyl.y,cyl.z);
end
h_sph=surf(sph.x,sph.y,sph.z);

set(h_sph,'LineStyle','none','FaceColor',0.8*[1 1 1],...
    'FaceLighting','gouraud','AmbientStrength',0.5,...
    'Facealpha',0.3,'Facecolor',[0.8 0.8 0.8]);
set(h_cyl,'Linestyle','none',...
    'FaceLighting','gouraud','AmbientStrength',0.5,...
    'Facealpha',0.5,'Facecolor','red');

light('Position',[0 0 100],'Style','infinite');
material dull;
axis equal;
axis off
xlabel('x')
ylabel('y')
zlabel('z')

tcs = patch('faces', TF, 'vertices', TV);
set(tcs, 'facec', 'flat');            % Set the face color flat
set(tcs, 'FaceVertexCData', TC);       % Set the color (from file)
set(tcs, 'EdgeColor','none');         % Set the edge color

line([0 1],[0 0],[0 0],'color','k','linewidth',3);
line([0 0],[0 1],[0 0],'color','k','linewidth',3);
line([0 0],[0 0],[0 1],'color','k','linewidth',3);

% convert the body fixed vector to the inertial frame
sen_inertial = zeros(length(tspan),3);

for ii = 1:length(tspan)
    sen_inertial(ii,:) = (R_b2i(:,:,ii)*constants.sen)';
end
sen_inertial_start = constants.R0*constants.sen;
sen_inertial_end = constants.Rd*constants.sen;
% plot path of body vector in inertial frame
plot3(sen_inertial_start(1),sen_inertial_start(2),sen_inertial_start(3),'go','markersize',10,'linewidth',2)
plot3(sen_inertial_end(1),sen_inertial_end(2),sen_inertial_end(3),'gx','markersize',10,'linewidth',2)

% pause to allow user to adjust the graphics window
fprintf('\nADJUST THE VIEW IN THE FIGURE WINDOW\n\n')
keyboard;

switch type
    case 'gif'
f = getframe;
[im,map] = rgb2ind(f.cdata,256,'nodither');
    case 'movie'
%         M(1:length(tspan))= struct('cdata',[],'colormap',[]);
    nFrames = length(tspan);
    vidObj = VideoWriter([filename '.avi']);
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end
hold on
% line([0 boresight(1)],[0 boresight(2)],[0 boresight(3)],'color','k','linewidth',3);
for ii = 1:10:length(tspan)
    
    nTV = TV *R_b2i(:,:,ii)';
    
    set(tcs,'Vertices',nTV);
    
    bore_handle = line([0 sen_inertial(ii,1)],[0 sen_inertial(ii,2)],[0 sen_inertial(ii,3)],'color','k','linewidth',3);
    plot3(sen_inertial(1:ii,1),sen_inertial(1:ii,2),sen_inertial(1:ii,3),'b','linewidth',3);
    
    drawnow
    switch type
        case 'gif'
            
            frame = getframe(1);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            outfile = [filename '.gif'];
            
            % On the first loop, create the file. In subsequent loops, append.
            if ii==1
                imwrite(imind,cm,outfile,'gif','DelayTime',0,'loopcount',inf);
            else
                imwrite(imind,cm,outfile,'gif','DelayTime',0,'writemode','append');
            end
        case 'movie'
%             M(ii)=getframe(gcf,[0 0 560 420]); % leaving gcf out crops the frame in the movie.
            writeVideo(vidObj,getframe(gca));
        otherwise
            
            
    end
    delete(bore_handle)
end


% Output the movie as an avi file
switch type
    case 'gif'
        
        
    case 'movie'
%         movie2avi(M,[filename '.avi']);
    close(vidObj);
    
    otherwise
        
end

fprintf('\nFINISHED ANIMATION\n\n')
fprintf('\nLOOK FOR %s in current directory\n\n', filename)

