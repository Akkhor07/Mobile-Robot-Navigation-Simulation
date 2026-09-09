function obstacles = define_obstacles()
% Hand-placed obstacle layout spanning an 8x8 m world, mixing shape types
% (circle, rectangle, triangle, diamond) similar to the reference thumbnail.
    obstacles = struct('type',{},'center',{},'radius',{},'w',{},'h',{},'verts',{});

    obstacles(end+1) = struct('type','circle','center',[2.0 2.1],'radius',0.40, ...
                               'w',[],'h',[],'verts',[]);
    obstacles(end+1) = struct('type','rect','center',[4.2 1.4],'radius',[], ...
                               'w',1.0,'h',0.55,'verts',[]);
    obstacles(end+1) = struct('type','circle','center',[6.0 5.9],'radius',0.5, ...
                               'w',[],'h',[],'verts',[]);
    obstacles(end+1) = struct('type','rect','center',[1.6 4.6],'radius',[], ...
                               'w',0.6,'h',1.2,'verts',[]);
    obstacles(end+1) = struct('type','circle','center',[4.1 4.1],'radius',0.35, ...
                               'w',[],'h',[],'verts',[]);

    tri = [5.6 3.2; 6.5 3.2; 6.05 4.1];
    obstacles(end+1) = struct('type','poly','center',[],'radius',[],'w',[],'h',[],'verts',tri);

    dia = [2.6 5.9; 3.15 6.45; 2.6 7.0; 2.05 6.45];
    obstacles(end+1) = struct('type','poly','center',[],'radius',[],'w',[],'h',[],'verts',dia);

    tri2 = [6.6 1.2; 7.4 1.2; 7.0 2.0];
    obstacles(end+1) = struct('type','poly','center',[],'radius',[],'w',[],'h',[],'verts',tri2);
end

