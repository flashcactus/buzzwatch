$fs=0.05;
$fa=5;
eps=0.001;

mot_rad=3.02;
mot_suppthick=4-eps;
mot_suppaddw=0.8;
mot_suppoff=2;
mot_supphwdth=mot_rad+mot_suppaddw;
mot_len=10.1;
mot_shaft=6;
mot_wires=1.7;
mot_total_len=mot_len+mot_shaft+mot_wires;

brd_bthick=1.5;
brd_topthick=5.5;
brd_botthick=2;
brd_lx=27.53;
brd_ly=18.8;
brd_slack=0.5;
brd_vslack=0.2;

brd_hrad=0.5;
brd_hox=1;
brd_hoy=1;
brd_hlx=25.5;
brd_hly=16.75;

brd_rheo_offset=[4.72,8.31,3.38];
brd_rheo_rad=7;
brd_rheo_thick=1.2;
brd_rheo_slack=0.5;

brd_batt_dia=12.5;
brd_batt_thick=3.5;//incl. wire
brd_batt_x=21.25;
brd_batt_y=9.25;

brd_sw_w=2.7;
brd_sw_h=1.64;
brd_sw_d=1;
brd_sw_offset=[19.05,1,0];

sw_hole_h=1.2;
sw_hole_w=2;
sw_hole_skirt=0.4;

case_thick=0.8;
case_pin_skirt=0.5;
case_slitslack=0.1;

nut_hole_add=0.2;
nut_sz=nut_hole_add+5.5;
nut_thk=2.4;
nut_depth=2.5;

bolt_hole_add=0.15;
bolt_dia=bolt_hole_add+3;
bolt_len=8;
bolt_head_len=2;
bolt_head_td=bolt_hole_add+5.52;
bolt_depth=1;
nb_mass_thk=1.2;

lid_skirt_h=1;
lid_skirt_w=0.8;
lid_bh_base=1.5;
lid_bh_foot=0.8;
lid_bh_x=20;

lid_lip_thk=0.8;
lid_lip_depth=0.8;
lid_lgrip_thk=0.8;
      
strap_pindst=2;
strap_pindia=1;
strap_tabdia=3;
strap_tabthick=4;

module mynut() { nut(nut_sz,nut_thk); }
module mybolt() { bolt(bolt_dia,bolt_len,bolt_head_len,bolt_head_td,bolt_dia); }

int_box=[
    brd_lx+brd_slack*2,
    brd_ly+brd_slack*2+mot_supphwdth*2,
    brd_topthick+brd_botthick+brd_vslack*2
    ];
int_box_off=[
    -brd_slack,
    -brd_slack,
    -brd_vslack-brd_botthick
    ];

ext_box=int_box+[2,2,1]*case_thick;//keep the top open
ext_box_off=int_box_off-[1,1,1]*case_thick;


case_lid_slack=0.25;//min distance between lid & everything else


motor_off=[brd_slack+mot_total_len+0.5,
          brd_ly+brd_slack+mot_suppaddw/2-eps,
          -brd_botthick-brd_vslack-eps
          ];



bn_hoff=[
    (motor_off[0]+int_box[0]+int_box_off[0])/2,
    brd_ly+brd_slack+mot_supphwdth,
    0
    ];


//------------------

module nut(sz,thickn){
    intersection() {
        cube([sz,sz*2,thickn],true);
        rotate([0,0,60]) cube([sz,sz*2,thickn],true);
        rotate([0,0,-60])cube([sz,sz*2,thickn],true);
    }
}

module hnut(sz,thk,hole){
    difference(){
        nut(sz,thk);
        translate([0,0,-thk/2-eps]) 
            cylinder(thk+2*eps,hole/2,hole/2);
    }
}

module bolt(dia,tlen,hlen,htop,hbot) {
    union() {
        cylinder(tlen,dia/2,dia/2);
        translate([0,0,-eps]) 
            cylinder(hlen+eps,htop/2,hbot/2);
    }
}


module motcube(transl) {
    translate([0,0,transl])
    cube([mot_supphwdth*2,mot_supphwdth*2,mot_suppthick],true);
};

module motrings(){
    difference() {
        union() {
            motcube(mot_suppoff);
            motcube(mot_len-mot_suppoff);
        }
        #cylinder(mot_len, mot_rad, mot_rad, false);
    }
    translate([0,0,-mot_wires])
        %cylinder(mot_total_len,mot_rad-0.1,mot_rad-0.1);
}

module rmotrings(){
    translate([0,mot_supphwdth,mot_supphwdth])
        rotate(-90,[0,1,0])
            translate([0,0,mot_wires])
                motrings();
}


module rmotcradle(){
    difference() {
        rmotrings();
        translate([0,mot_supphwdth,mot_supphwdth+mot_rad]) {
            cube(2*[mot_total_len,mot_rad,mot_rad],true);
        }
    }
}

module rmotkeepout(slack){
    translate([0,mot_supphwdth,mot_supphwdth])
        rotate(-90,[0,1,0])
            translate([0,0,mot_total_len/2])
                cube([2,2,0]*(mot_supphwdth+slack) + [0,0,mot_total_len], true);
}


module brdhole(x,y,rad,top,bot) {
    translate(
      [brd_hox+brd_hlx*x,
       brd_hoy+brd_hly*y,
       bot]
    )
        cylinder(top-bot,rad,rad);
}

module mainbrd(slack, rslack, zslack) { 
    union() {
        difference() {//the board
            translate([-slack,-slack,0])
                cube([brd_lx+slack*2,brd_ly+slack*2,brd_bthick+zslack],false);
            brdhole(0,0,brd_hrad,brd_bthick+0.1,-0.1);
            brdhole(0,1,brd_hrad,brd_bthick+0.1,-0.1);
            brdhole(1,0,brd_hrad,brd_bthick+0.1,-0.1);
            brdhole(1,1,brd_hrad,brd_bthick+0.1,-0.1);
        }
        //rheo
        translate(brd_rheo_offset-[0,0,rslack]) {
            cylinder(brd_rheo_thick+rslack*2,brd_rheo_rad+rslack,brd_rheo_rad+rslack);
        }
        //button
        translate(brd_sw_offset+[-brd_sw_w/2, 0, -brd_sw_h]) {
            cube([brd_sw_w,brd_sw_d,brd_sw_h],false);
        }
        //batt
        batt_off=[brd_batt_x,brd_batt_y,brd_bthick];
        translate(batt_off)
            cylinder(brd_batt_thick,brd_batt_dia/2,brd_batt_dia/2);
    }
}

module lidlip(thickness,depth){
    //lid grip lip
    llip_dim = [depth+2*eps,
                int_box[1]+2*eps,
                thickness];
    llip_off = int_box_off+[-eps,-eps,int_box[2]-llip_dim[2]];
    
    translate(llip_off)
        cube(llip_dim);
}

module cbox(){        
    pin_brad = brd_hrad+case_pin_skirt;
    pin_trad = brd_hrad;
    pin_top = brd_bthick+0.1;
    pin_bot = int_box_off[2]-0.01;
    
    slit_dim = [case_thick,
                brd_hly,
                brd_bthick+case_thick-int_box_off[2]
                ];
    slit_off = int_box_off+[0,brd_hoy-int_box_off[1],0];

    
    union() {
        difference() {
            //outer shell
            translate(ext_box_off)
                cube(ext_box);
            //main cavity
            translate(int_box_off)
                cube(int_box+[0,0,0.01]);
        }
        
        //pins
        brdhole(1,0,pin_brad,0,pin_bot);
        brdhole(1,0,pin_trad,pin_top,-0.1);
        brdhole(1,1,pin_brad,0,pin_bot);
        brdhole(1,1,pin_trad,pin_top,-0.1);
        
        //potside slit mass
        translate(slit_off)
            cube(slit_dim);
        
        //lid grip
        lidlip(lid_lip_thk,lid_lip_depth);

        
        //screw mass
        bm_orad = bolt_head_td/2+nb_mass_thk;
        bm_srad = bolt_dia/2+nb_mass_thk;
        translate(bn_hoff+[0,0,ext_box_off[2]+eps]){
            cylinder(bolt_depth,bm_orad,bm_orad);
                translate([0,0,bolt_depth-eps])
                    cylinder(bolt_head_len,bm_orad,bm_srad);
        }
    }
}

module case(){
    union(){
        difference() {
            cbox();
            
            //hole for rheo
            mainbrd(brd_slack,brd_rheo_slack,0);
            
            //hole for button
            translate(brd_sw_offset+[0,0,-sw_hole_h/2-sw_hole_skirt])
                cube([sw_hole_w,8,sw_hole_h],true);
            
            //hole for screw
            translate(bn_hoff+[0,0,ext_box_off[2]-eps]){
                cylinder(bolt_depth+2*eps,bolt_head_td/2,bolt_head_td/2);
                translate([0,0,bolt_depth+eps])
                    #mybolt();
            }
        }
        
        //motor
        translate(motor_off){
            rmotcradle();
        }
        
        translate(ext_box_off+[0,0,strap_tabdia/2])
            rotate (-90,[0,1,0]) 
                strappair(
                  ext_box[1],
                  strap_tabdia,
                  strap_pindst,
                  strap_pindia,
                  strap_tabthick
                );
        
         translate(ext_box_off+[ext_box[0],0,strap_tabdia/2])
            rotate (90,[0,1,0]) 
                strappair(
                  ext_box[1],
                  strap_tabdia,
                  strap_pindst,
                  strap_pindia,
                  strap_tabthick
                );
    }
}

module lid(){
    lid_dim = [ext_box[0],ext_box[1],case_thick];
    lid_off = ext_box_off+[0,0,ext_box[2]];
    //skirt mass
    lsk_dim = lid_dim+[0,0,lid_skirt_h-case_thick]+[-2,-2,1]*eps;
    lsk_off = ext_box_off+[eps,eps,(ext_box[2]-lid_skirt_h)];
    //perimeter skirt 
    lsksub_dim = lsk_dim-[2,2,0]*(lid_skirt_w+case_thick)+[0,0,2*eps];
    lsksub_off = lsk_off+[1,1,0]*(lid_skirt_w+case_thick)+[0,0,-eps];
    //case lip
    lidgrip_dim = [
        case_thick+3*lid_lip_depth-2*eps,
        lid_dim[1]-2*eps,
        lid_lgrip_thk+lid_lip_thk+case_slitslack+eps
    ];
    lidgrip_off = lid_off+[0,0,-lidgrip_dim[2]]+[1,1,1]*eps;
    
    nut_off = bn_hoff+[0,0,lid_off[2]+case_thick];

    bh_hght = lid_off[2]+lid_skirt_h-brd_bthick;
    bh_z = lid_off[2]+lid_skirt_h-bh_hght;
    
    difference(){
        union(){
            //lid top
            translate(lid_off) cube(lid_dim);
            //skirt
            difference () {
                    translate(lsk_off) cube(lsk_dim);
                    translate(lsksub_off) cube(lsksub_dim);
                    translate(brd_rheo_offset) cube(brd_rheo_rad*2,true);
            }
            //lip-grip
            difference(){ 
                translate(lidgrip_off) 
                    cube(lidgrip_dim);
                translate(brd_rheo_offset) 
                    cube(brd_rheo_rad*2,true);
                lidlip(lid_lip_thk+case_slitslack,lid_lip_depth+case_slitslack);
            }

            translate([0,0,eps]) intersection(){
                translate(int_box_off)
                    cube(int_box);
                union(){
                    //board-holders
                    translate([lid_bh_x,0,bh_z])
                        cylinder(bh_hght+eps,lid_bh_foot,lid_bh_base);
                    translate([lid_bh_x,brd_ly,bh_z])
                        cylinder(bh_hght+eps,lid_bh_foot,lid_bh_base);
                
                            //nut mass
                    translate(nut_off) 
                        nut(nut_sz+nb_mass_thk, (nut_depth+nb_mass_thk)*2);                  
                }           
            }
            
            
            
        }
        
        case();
        
        translate(motor_off){
            rmotkeepout(case_lid_slack);
        }
        
        translate(nut_off){ 
            nut(nut_sz,nut_depth*2);
            translate([0,0,-bolt_len]) 
                mybolt();
            translate([0,0,nut_thk/2-nut_depth])
                %hnut(nut_sz,nut_thk,bolt_dia);
        }
        
    }
}

module straptab(wid,dist,holedia,thickn) {
    translate([-wid/2,-thickn/2,-eps])
        difference() {
            union() {
                cube([wid,thickn,dist+eps]);
                translate([wid/2,0,dist+eps]){
                    rotate(-90,[1,0,0]) {
                            cylinder(thickn,wid/2,wid/2);
                    }
                }
            }
            
            translate([wid/2,-eps,dist+eps]){
                rotate(-90,[1,0,0]) {
                        cylinder(thickn+eps*2,holedia/2,holedia/2);
                }
            }
        }
}    

module strappair(pwidth,twidth,pdist,pindia,tthick) {
    union() {
        translate([0,tthick/2,0]) straptab(twidth,pdist,pindia,tthick);
        translate([0,pwidth-tthick/2,0]) straptab(twidth,pdist,pindia,tthick);
    }
}




*rmotcradle();

translate(-ext_box_off){
    *lid();
    case();
    *mainbrd(0.4,0,0);
}


