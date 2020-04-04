from bpy import context, data, ops
import math
from mathutils import geometry

def def_track_1():
    pts=[] 
    #pts.append([   0.0,   0.0  , 0.0 , 30.0 , 20.0])  # x,y,z,turn0,turn1
    #pts.append([   0.0,  160.0 , 0.0 , 20.0 , 20.0])
    #pts.append([ 100.0,  200.0 , 0.0 , 20.0 , 20.0])
    #pts.append([ 180.0,   0.0  , 5.0 , 20.0 , 10.0])
    #pts.append([-180.0, -50.0  , 5.0 , 10.0 , 10.0])
    #pts.append([-180.0, -160.0 , 0.0 , 10.0 , 20.0])
    #pts.append([   0.0, -200.0 , 0.0 , 10.0 , 30.0])
#Circuit ellipse
#first cadran
    pts.append([ 270.0,   0.0  , 0.0 , 20.0 , 20.0])
    pts.append([ 250.0,  50.0  , 0.0 , 20.0 , 20.0])
    pts.append([ 150.0,  90.0  , 0.0 , 20.0 , 20.0])
    pts.append([   0.0, 100.0  , 0.0 , 20.0 , 20.0])
#second one
    pts.append([ -150.0,  90.0  , 0.0 , 20.0 , 20.0])
    pts.append([ -250.0,  50.0  , 0.0 , 20.0 , 20.0])
    pts.append([ -270.0,   0.0  , 0.0 , 20.0 , 20.0])
#third
    pts.append([ -250.0,  -50.0  , 0.0 , 20.0 , 20.0])
    pts.append([ -150.0,  -90.0  , 0.0 , 20.0 , 20.0])
    pts.append([    0.0, -100.0  , 0.0 , 20.0 , 20.0])
#fourth
    pts.append([ 150.0,  -90.0  , 0.0 , 20.0 , 20.0])
    pts.append([ 250.0,  -50.0  , 0.0 , 20.0 , 20.0])       
 
    # define the number of basis points fro the circular bezier curve
    # ncut is the number of subdivision
    n0=len(pts)
    ncut = 1
    over_sample = 4 # define over sampling factor to have a smoother curve
    while True:
        npt_bez = 4*(ncut+1)  # compute the number of beziers points
        if npt_bez > over_sample*n0:
            break  # exit if subdivision is enough
        ncut += 1
    npt = npt_bez

    # each side of the polygon is reduce at its beginning and at its end to let space for the curves
    # a vertex of the polygon is defined by 5 values : xv,yv,zv,turn0,turn1
    #  (xv,yv,zv) are the coordinates of the vertex, 
    #  turn0 is the place left for the curve before reaching the vertex
    #  turn1 is the place left for the curve after passing the vertex
    # we compute points (x,y,z) at the begining and at the end of each curve
    # also compute the left (xhl,yhl,zhl) and right (xhr,yhr,zhr) handlers to define the tangent to the bezier curve 
    x,y,z,xhl,yhl,zhl,xhr,yhr,zhr = def_track_curves(pts)

    # define how many points are on each segment of the track
    t_cnt = def_points_in_segments(npt,x,y)

    # compute all points that will  define the bezier curve of the track
    lpts = []
    n = len(x)
    for i0 in range(n):
        i1 = (i0+1) % n
        new_pts = geometry.interpolate_bezier(
            (x[i0],y[i0],z[i0]),
            (xhr[i0],yhr[i0],zhr[i0]),
            (xhl[i1],yhl[i1],zhl[i1]),
            (x[i1],y[i1],z[i1]),
            t_cnt[i0]+1) # trick : compute one more points to avoid duplicate of the first
        for ipt in range(len(new_pts)-1):
            pt =new_pts[ipt+1]
            lpts.append([pt.x,pt.y,pt.z])
            
    nptl = len(lpts)
    return ncut,nptl,lpts

def def_track_curves (pts):
    n0=len(pts)
    xhl = []
    yhl = []
    zhl = []
    xhr = []
    yhr = []
    zhr = []
    x=[]
    y=[]
    z=[]
    for i0 in range(n0):
        i1 = (i0+1) % n0
        x0 = pts[i0][0]
        y0 = pts[i0][1]
        z0 = pts[i0][2]
        x1 = pts[i1][0]
        y1 = pts[i1][1]
        z1 = pts[i1][2]
        l = math.sqrt((x0-x1)**2+(y0-y1)**2+(z0-z1)**2)
        dla = pts[i0][4]
        dlb = pts[i1][3]
        xa = x0+dla*(x1-x0)/l
        ya = y0+dla*(y1-y0)/l
        za = z0+dla*(z1-z0)/l
        xb = x0+(l-dlb)*(x1-x0)/l
        yb = y0+(l-dlb)*(y1-y0)/l
        zb = z0+(l-dlb)*(z1-z0)/l
        dla /= 2.0
        dlb /= 2.0
        xhla = xa-dla*(x1-x0)/l
        yhla = ya-dla*(y1-y0)/l
        zhla = za-dla*(z1-z0)/l
        xhra = xa+dla*(x1-x0)/l
        yhra = ya+dla*(y1-y0)/l
        zhra = za+dla*(z1-z0)/l
        xhlb = xb-dlb*(x1-x0)/l
        yhlb = yb-dlb*(y1-y0)/l
        zhlb = zb-dlb*(z1-z0)/l
        xhrb = xb+dlb*(x1-x0)/l
        yhrb = yb+dlb*(y1-y0)/l
        zhrb = zb+dlb*(z1-z0)/l
        x.append(xa)
        y.append(ya)
        z.append(za)
        xhl.append(xhla)
        yhl.append(yhla)
        zhl.append(zhla)
        xhr.append(xhra)
        yhr.append(yhra)
        zhr.append(zhra)
        x.append(xb)
        y.append(yb)
        z.append(zb)
        xhl.append(xhlb)
        yhl.append(yhlb)
        zhl.append(zhlb)
        xhr.append(xhrb)
        yhr.append(yhrb)
        zhr.append(zhrb)
    return x,y,z,xhl,yhl,zhl,xhr,yhr,zhr

def def_points_in_segments(npt,x,y):
    n = len(x)
    t_len = []
    l_tot = 0.0
    for i0 in range(n):
        i1 = (i0+1) % n
        x0 = x[i0]
        y0 = y[i0]
        x1 = x[i1]
        y1 = y[i1]
        dl = math.sqrt((x0-x1)**2.0+(y0-y1)**2.0)
        t_len.append(dl)
        l_tot += dl
    t_cnt = []
    n1 = 0
    npt1 = npt-2*n
    for i in range(n):
        nadd = int(t_len[i]/l_tot*npt1)+2
        t_cnt.append(nadd)
        n1 += nadd
    if n1 != npt:
        for i in range(n):
            t_cnt[i] += 1
            n1 += 1
            if n1 == npt:
                break
    return t_cnt

def get_length(obj):
    lcurv = 0.0
    mesh = obj.to_mesh()
    #print (dir(mesh))
    vs = mesh.edges.values()
    #print (len(mesh.edges),len(vs),type(vs[0]),dir(vs[0]))
    for v in vs:
        x0 = mesh.vertices[v.vertices[0]].co.x
        y0 = mesh.vertices[v.vertices[0]].co.y
        z0 = mesh.vertices[v.vertices[0]].co.z
        x1 = mesh.vertices[v.vertices[1]].co.x
        y1 = mesh.vertices[v.vertices[1]].co.y
        z1 = mesh.vertices[v.vertices[1]].co.z
        dlcurv = math.sqrt((x0-x1)**2+(y0-y1)**2+(z0-z1)**2)
        lcurv += dlcurv
        #print (dlcurv,lcurv)
    return lcurv

def define_cylinder (name, diameter, length, x, y, z, angx, angy , angz):
    # Create a simple cylinder.
    ops.mesh.primitive_cylinder_add(radius = diameter/2.0, depth=length)
    # Get the cylinder object and rename it.
    cyl = context.object
    cyl.name = name 
    # Change the orientation of the cylinder.
    cyl.rotation_euler = (angx*deg2rad, angy*deg2rad, angz*deg2rad)
    # Change the location of the cylinder.
    cyl.location = (x, y, z)
    #print (dir(cyl))
    return cyl

# define a gate using 2 vertical cylinders on top of which we put an horizontal
# cylinder. The gate is built using the union function of CSG (constructive solid geometry) 
def create_gate(name):
    cyl1 = define_cylinder ("Gate1", 0.5, 3.0 ,-5.0, 0.0, 1.5, 0.0, 0.0 , 0.0)
    cyl2 = define_cylinder ("Gate2", 0.5, 3.0 , 5.0, 0.0, 1.5, 0.0, 0.0 , 0.0)
    cyl3 = define_cylinder ("Gate3", 0.5, 9.5 , 0.0, 0.0, 2.75, 0.0, 90.0 , 0.0)
    cyl1.select_set(False)
    cyl2.select_set(False)
    cyl3.select_set(True)
    csg2 = cyl3.modifiers.new(name="csg2", type="BOOLEAN")
    csg2.operation = "UNION"
    csg2.object=cyl2
    ops.object.modifier_apply(apply_as='DATA', modifier='csg2')
    csg1 = cyl3.modifiers.new(name="csg1", type="BOOLEAN")
    csg1.operation = "UNION"
    csg1.object=cyl1
    ops.object.modifier_apply(apply_as='DATA', modifier='csg1')
    gate = context.active_object
    gate.select_set(False)
    cyl1.select_set(True)
    cyl2.select_set(True)
    ops.object.delete()
    gate.select_set(True)
    gate.name = name
    gate.rotation_euler = (90.0*deg2rad, 90.0*deg2rad, 0.0*deg2rad)
    #ops.transform.resize(value=(10.0, 10.0, 10.0))
    return gate

 
deg2rad = math.pi/180.0

# Get the track waypoints
ncut,n,lpts = def_track_1()


# Create a bezier circle and enter edit mode.
ops.curve.primitive_bezier_circle_add (radius=1.0,
                                      location=(0.0, 0.0, 0.0),
                                      enter_editmode=False)
                                      
# Subdivide the curve by a number of cuts
ops.object.mode_set(mode='EDIT')
ops.curve.subdivide(number_cuts=ncut)

# Return to object mode.
ops.object.mode_set(mode='OBJECT')


curve = context.active_object
curve.name = 'TrackCenter'
pts = curve.data.splines[0].bezier_points
npt = len(pts)

xmin = lpts[0][0]
ymin = lpts[0][1]
zmin = lpts[0][2]
xmax = xmin
ymax = ymin
zmax = zmin
for i in range(len(pts)):
    pt = pts[i]
    x = lpts[i][0]
    y = lpts[i][1]
    z = lpts[i][2]
    pt.co.x = x
    pt.co.y = y
    pt.co.z = z
    print (i,pt.co,pt.handle_left,pt.handle_left_type)
    if x > xmax:
        xmax = x
    if x < xmin:
        xmin = x
    if y > ymax:
        ymax = y
    if y < ymin:
        ymin = y
    if z > zmax:
        zmax = z
    if z < zmin:
        zmin = z
print (npt,"min,max",xmin,xmax,ymin,ymax,zmin,zmax)

ltrk_bez = get_length(curve)
print ("track length",ltrk_bez)

# define the basis track element that we slided along the track
# it is a plane of 1x5 blender units (a cube could have been used too)
ops.mesh.primitive_plane_add (location=(0.0, 0.0, 0.0))
base = context.active_object
base.name = 'TrackElement'
ops.transform.resize(value=(1.0, 5.0, 1.0)) # set 5 units width
# add an array modifier, it will add as many elements as needed to cover
# the curve
ma = base.modifiers.new(name='mod_array_' + base.name, type="ARRAY")
ma.fit_type = "FIT_CURVE"
ma.curve = curve
ma.relative_offset_displace = (1.0,0.0,0.0)
# add a curve modifier to say what curve must be followed
mc = base.modifiers.new(name='mod_curve_' + base.name, type="CURVE")
mc.object = curve
# apply the modifiers
ops.object.modifier_apply(apply_as='DATA', modifier='mod_array_TrackElement')
ops.object.modifier_apply(apply_as='DATA', modifier='mod_curve_TrackElement')


add_gates = True
# Create gates.
# we use the same principle , we place the gates along the curve every 100 units
# by applying array and curve modifiers to a basic gate (created once)
if add_gates:
    # create one basic gate 
    gates = create_gate("GateElement")
    # Append modifiers.
    array_mod = gates.modifiers.new(name='mod_array_'+gates.name, type='ARRAY')
    curve_mod = gates.modifiers.new(name='mod_curve_'+gates.name, type='CURVE')
    # Array modifier properties.
    array_mod.fit_type = 'FIT_CURVE'
    array_mod.curve = curve
    array_mod.use_relative_offset = True
    distance_between_gates = 100.0
    array_mod.relative_offset_displace = (0.0, distance_between_gates, 0.0)
    # Curve modifier properties.
    curve_mod.object = curve
    curve_mod.deform_axis = 'POS_X'

