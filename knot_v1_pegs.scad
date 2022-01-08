$fs = 0.1;

step = 1;
width = 20;
segment_width = 0.1;
cutter_width = 0.15;
strip_length = 2500;

px = 3;
py = 2;
pz = 3;
cx = 3;
cy = 4;
cz = 2;

function curve(t) = [
    (cx*cos(px*t)+cos(t)),
    (cy*sin(py*t)),
    (cz*sin(pz*t)+sin(t))
];

function curve_length(i) = i <= 0 ? 0 : norm(curve(i) - curve(i+1)) + curve_length(i-1);
clen = curve_length(360);
echo("Original curve length:", clen);

function f(t) = curve(t)/clen*strip_length;

function f_length(i) = i <= 0 ? 0 : norm(f(i) - f(i+1)) + f_length(i-1);
echo("Mid point length: ", f_length(360));

function speed(t)  = [
    -cx*px*sin(px*t)-sin(t),
    cy*py*cos(py*t),
    cz*pz*cos(pz*t)+cos(t)
];

function acceleration(t) = [
    -cx*px*px*cos(px*t) - cos(t),
    -cy*py*py*sin(py*t),
    -cz*pz*pz*sin(pz*t) - sin(t)
];

function localx(t) = speed(t)/norm(speed(t));
function centripetal(t) = acceleration(t)-(acceleration(t)*localx(t))*localx(t);
function localy(t) = centripetal(t)/norm(centripetal(t));
function localz(t) = cross(localx(t),localy(t));

function top_loc(t) = f(t) + localz(t)*4;
function top_length(i) = i <= 0 ? 0 : norm(top_loc(i) - top_loc(i+1)) + top_length(i-1);
echo("Top strip length: ",top_length(360));

function bottom_loc(t) = f(t) - localz(t)*4;
function bottom_length(i) = i <= 0 ? 0 : norm(bottom_loc(i) - bottom_loc(i+1)) + bottom_length(i-1);
echo("Bottom strip length: ",bottom_length(360));

function matrix(t) = [
    [localx(t)[0],localy(t)[0],localz(t)[0],f(t)[0]],
    [localx(t)[1],localy(t)[1],localz(t)[1],f(t)[1]],
    [localx(t)[2],localy(t)[2],localz(t)[2],f(t)[2]],
    [0,0,0,1]
];

function inverse_matrix(t) = [
    concat(localx(t),[0]),
    concat(localy(t),[0]),
    concat(localz(t),[0]),
    [0,0,0,1]
];

module body(begin, end, add_pegs) {
    color("BurlyWood")
    union() { 
        if(add_pegs) {
            multmatrix(matrix(end+step))
            pegs();
        }
        
        difference() {
            union() {
                for(a = [begin:step:end]) {
                    hull() {
                        multmatrix(matrix(a))
                        cube(size = [segment_width,14,width], center=true);
                        
                        multmatrix(matrix(a+step))
                        cube(size = [segment_width,14,width], center=true);
                    }
                }
            }  
            
            if(add_pegs) {
                multmatrix(matrix(begin))
                pegs_cutter();
            }
            
            union() {
                for(a = [begin:step:end]) {
                    
                    hull() {
                        multmatrix(matrix(a))
                        cube(size = [cutter_width,4,11], center=true);
                        
                        multmatrix(matrix(a+step))
                        cube(size = [cutter_width,4,11], center=true);
                    }
                }
            }
            
            union(){
                for(a = [begin:step:end]) {
                    hull() {
                        multmatrix(matrix(a))
                        arc(width/2-2,cutter_width);
                        
                        multmatrix(matrix(a+step))
                        arc(width/2-2,cutter_width);
                    }
                    hull() {
                        multmatrix(matrix(a))
                        mirror([0,1,0])
                        arc(width/2-2,cutter_width);
                        
                        multmatrix(matrix(a+step))
                        mirror([0,1,0])
                        arc(width/2-2,cutter_width);
                    }
                }
            }
            
            union() {
                for(a = [begin:step:end]) {
                    if(a % 20 == 10) {
                        hull() {
                            multmatrix(matrix(a-0.5))
                            cube(size = [cutter_width,16,width-8], center=true);
                            
                            multmatrix(matrix(a+0.5))
                            cube(size = [cutter_width,16,width-8], center=true);
                        }
                    }
                }
            }
        }
    }
}



module arc_cutter(r,h) {
    translate([0,-4,0])
    rotate(a=[90,0,0])
    rotate(a=[0,90,0])
    cylinder(h=h,r=r, center=true);
}


module cover(m, begin, end) {
    difference() {
        union(){
            for(a = [begin:step:end]) {
                hull() {
                    multmatrix(matrix(a))
                    mirror([0,m,0])
                    arc(width/2-2,segment_width);
                    
                    multmatrix(matrix(a+step))
                    mirror([0,m,0])
                    arc(width/2-2,segment_width);
                }
            }
        }
        union(){
            for(a = [begin:step:end]) {
                hull() {
                    multmatrix(matrix(a))
                    mirror([0,m,0])
                    arc_cutter(width/2-4,cutter_width);
                    
                    multmatrix(matrix(a+step))
                    mirror([0,m,0])
                    arc_cutter(width/2-4,cutter_width);
                }
            }
        }
    }
}



part_length = 20;

for(j = [0:1:0]){
    translate([0,j*30,0])
    rotate([-90,0,0])
    multmatrix(inverse_matrix((j+0.5)*part_length))
    translate(-f((j+0.5)*part_length))
    {
        body(j*part_length, (j+1)*part_length-step, true);
    }
}

/*
body(0,360, false);
covers(0,360);
*/
/*
color("red",0.2)
body(0*part_length, (0+1)*part_length-step);

color("green",0.2)
body(1*part_length, (2+1)*part_length-step);
*/

peg_spacing = 0.2;

module peg(s) {
    translate([0,5.5,-2]){
        translate([-6,0-s,0+peg_spacing-s])
        cube([12+s,2+2*s,4-2*peg_spacing+2*s]);
        translate([5,2,+peg_spacing-s])
        cylinder(h=4-2*peg_spacing+2*s,r=1+s);
    }
}

module pegs() {
    rotate([90,0,0]) {
        peg(0);
        mirror([0,1,0])
        peg(0);
    }
}

module pegs_cutter() {
    rotate([90,0,0]) {
        peg(peg_spacing);
        mirror([0,1,0])
        peg(peg_spacing);
    }
}




