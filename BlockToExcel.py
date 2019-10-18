import rhinoscriptsyntax as rs
import scriptcontext as sc
import Rhino
import Rhino, math, time
import Rhino.DocObjects.ObjectType as OT

import clr
import sys
#======================Methods=====================================#
def RotatedPlaneArray(plane,tot_ang,divs,axis):
    #creates an array of planes rotated in increments around an axis
    #tot_ang=total array inc. angle (rads); divided into divs number of divisions
    #included angle is interval from -tot_ang/2 to +tot_ang/2
    #number of output planes is divs; number of angle divisions is divs-1
    out_planes=[]
    plane.Rotate(-tot_ang*0.5,axis)
    out_planes.append(Rhino.Geometry.Plane(plane))
    inc=tot_ang/(divs-1)
    for i in range(divs-1):
        plane.Rotate(inc,axis)
        out_planes.append(Rhino.Geometry.Plane(plane))
    return out_planes
def RotatePlaneArray3D(view_plane,tot_ang,divs):
    #generate a 3D array of refinement planes (works with narrow angles)
    out_planes=[]
    #use RotatedPlaneArray to generate 'horizontal' left-right array (yaw)
    yaw_planes=RotatedPlaneArray(view_plane,tot_ang,divs,view_plane.ZAxis)
    for y_plane in yaw_planes:
        #use RotatedPlaneArray to generate side-to-side 'tilt' array (roll)
        roll_planes=RotatedPlaneArray(y_plane,tot_ang,divs,y_plane.YAxis)
        for r_plane in roll_planes:
            #use RotatedPlaneArray to generate up-down 'tilt' array (pitch)
            pitch_planes=RotatedPlaneArray(r_plane,tot_ang,divs,r_plane.XAxis)
            for p_plane in pitch_planes:
                out_planes.append(p_plane)
    return out_planes
def BoundingBoxPlane(objs,plane,ret_pts=False,accurate=True):
    """returns a plane-aligned bounding box in world coordinates
       - input geometry must be RhinoCommon geometry (not IDs)
       - adapted from python rhinoscriptsyntax rs.BoundingBox() code."""
    wxy_plane=Rhino.Geometry.Plane.WorldXY
    def __objectbbox(geom,xform):
        if isinstance(geom,Rhino.Geometry.Point):
            pt=geom.Location
            if xform: pt = xform * pt
            return Rhino.Geometry.BoundingBox(pt,pt)
        if xform: return geom.GetBoundingBox(xform)
        return geom.GetBoundingBox(accurate)
    
    xform = Rhino.Geometry.Transform.ChangeBasis(wxy_plane, plane)
    bbox = Rhino.Geometry.BoundingBox.Empty
    if type(objs) is list or type(objs) is tuple:
        for obj in objs:
            objectbbox = __objectbbox(obj, xform)
            bbox = Rhino.Geometry.BoundingBox.Union(bbox,objectbbox)
    else:
        objectbbox = __objectbbox(objs, xform)
        bbox = Rhino.Geometry.BoundingBox.Union(bbox,objectbbox)
    if not bbox.IsValid: return
    plane_to_world = Rhino.Geometry.Transform.ChangeBasis(plane,wxy_plane)
    if ret_pts:
        corners = list(bbox.GetCorners())
        for pt in corners: pt.Transform(plane_to_world)
        return corners
    else:
        box=Rhino.Geometry.Box(bbox)
        box.Transform(plane_to_world)
        return box
def Min3DBoundingBox(objs,init_plane,count,rel_stop,im_rep):
    #for non-planar or non-coplanar object(s)
    #get initial fast bb in init plane (World XY), plus volume to compare
    curr_bb=BoundingBoxPlane(objs,init_plane,False)
    curr_vol=curr_bb.Volume
    
    tot_ang=math.pi*0.5 #90 degrees for intial octant
    factor=0.1 #angle reduction factor for each successive refinement pass
    max_passes=20 #safety factor
    prec=sc.doc.ModelDistanceDisplayPrecision
    us=rs.UnitSystemName(abbreviate=True)
    
    #run intitial bb calculation
    xyz_planes=GenerateOctantPlanes(count)
    best_plane,curr_bb,curr_vol=MinBBPlane(objs,init_plane,xyz_planes,curr_bb,curr_vol)
    #report results of intial rough calculation
    if im_rep:
        print "Initial pass 0, volume: {} {}3".format(round(curr_vol,prec),us)
    #refine with smaller angles around best fit plane, loop until...
    for i in range(max_passes):
        prev_vol=curr_vol
        #reduce angle by factor, use refinement planes to generate array
        tot_ang*=factor
        ref_planes=RotatePlaneArray3D(best_plane,tot_ang,count)
        best_plane,curr_bb,curr_vol=MinBBPlane(objs,best_plane,ref_planes,curr_bb,curr_vol)
        vol_diff=prev_vol-curr_vol #vol. diff. over last pass, should be positive or 0
        #print "Volume difference from last pass: {}".format(vol_diff) #debug
        #check if difference is less than minimum "significant"
        #rel_stop==True: relative stop value <.01% difference from previous
        if rel_stop:
            if vol_diff<0.0001*prev_vol: break
        else:
            if vol_diff<sc.doc.ModelAbsoluteTolerance: break
        Rhino.RhinoApp.Wait()
        if im_rep:
            print "Refine pass {}, volume: {} {}3".format(i+1,round(curr_vol,prec),us)
        #get out of loop if escape is pressed
        if sc.escape_test(False):
            print "Refinement aborted after {} passes.".format(i+1)
            break
            
    return curr_bb,curr_vol,i+1
def GenerateOctantPlanes(count):
    tot_ang=math.pi*0.5 #90 degrees
    #generates an array of count^3 planes in 3 axes covering xyz positive octant
    yz_plane=Rhino.Geometry.Plane.WorldYZ
    dir_vec=Rhino.Geometry.Vector3d(1,0,0) #X axis
    x_planes=RotateCopyPlanes(tot_ang,count,yz_plane,dir_vec)
    dir_vec=Rhino.Geometry.Vector3d(0,-1,0) #-Y axis
    xy_planes=RotateCopyPlanes(tot_ang,count,x_planes,dir_vec)
    dir_vec=Rhino.Geometry.Vector3d(0,0,1) #Z axis
    xyz_planes=RotateCopyPlanes(tot_ang,count,xy_planes,dir_vec)
    return xyz_planes
def RotateCopyPlanes(tot_ang,count,init_planes,dir_vec):
    """takes a single plane or list of planes as input
    rotates/copies planes through angle tot_ang
    number of planes=count, number of angle divisions=count-1"""
    if isinstance(init_planes,Rhino.Geometry.Plane): init_planes=[init_planes]
    inc=tot_ang/(count-1)
    origin=Rhino.Geometry.Point3d(0,0,0)
    planes=[]
    objs=[]
    for i in range(count):
        for init_plane in init_planes:
            new_plane=Rhino.Geometry.Plane(init_plane)
            new_plane.Rotate(inc*i,dir_vec,origin)
            planes.append(new_plane)
    return planes
def MinBBPlane(objs,best_plane,planes,curr_box,curr_vol):
    """returns plane with smallest aligned bounding box volume
    from list of input objects, planes to test and initial compare volume
    best plane, volume, and bbox pass through if no better solution found"""
    for plane in planes:
        bb=BoundingBoxPlane(objs,plane,ret_pts=False)
        if bb.Volume<curr_vol:
            curr_vol=bb.Volume
            best_plane=plane
            curr_box=bb
    return best_plane,curr_box,curr_vol
#======================Methods=====================================#

#======================Get Excel Referance=========================#
sys.path.append(r"C:\Users\TishGhaith\Downloads\Microsoft.Office.Interop.Excel")
clr.AddReference ("Microsoft.Office.Interop.Excel.dll")

sys.path.append(r"C:\Users\TishGhaith\Desktop")
from Microsoft.Office.Interop import Excel
#======================Get Excel Referance=========================#

#======================Inputs======================================#
BoBoxIterationNum = 24
#======================Inputs======================================#

#======================Synchronizing To Excel======================#
ex = Excel.ApplicationClass()   
ex.Visible = True
ex.DisplayAlerts = False   
workbook = ex.Workbooks.Open('C:\Users\TishGhaith\Desktop\BlockScheduel.xlsx')
ws = workbook.Worksheets[1]
#======================Synchronizing To Excel======================#

#======================Get Rhino Objects===========================#
#Get all Blocks in the document
Blocks = rs.BlockNames()
BlockInstancesList = []
for i in Blocks:
    M = rs.BlockInstances(i, 0)
    BlockInstancesList.extend(M)
index = 0
for i in BlockInstancesList:
    if rs.IsBlockInstance(i):
        
        index+=1
        InstanceName = rs.ObjectName(i, rs.BlockInstanceName(i) + "_" + str(index))
        BlName = rs.ObjectName(i, InstanceName)
        #GettingBlockInsertionPoint
        BlockRefPt = rs.BlockInstanceInsertPoint(i)
        #Get The X,Y,Z Coordinates of the insertion point 
        PtX = rs.coerce3dpoint(BlockRefPt)[0]
        PtY = rs.coerce3dpoint(BlockRefPt)[1]
        PtZ = rs.coerce3dpoint(BlockRefPt)[2]
        #GettingBlockRotation
        Angle = 0
        
        Plane = rs.CreatePlane(rs.BlockInstanceInsertPoint(i), (1,0,0), (0,1,0))
        
        Bl01 = rs.CopyObject(i)
        Bl01 = rs.ExplodeBlockInstance(Bl01)[0]
        Bl02 = rs.coercebrep(Bl01)
        
        #Get The angle To The X
        ws.Cells[2+index, 6].Value2 = Angle
        BoBox = Min3DBoundingBox(Bl02,Plane,BoBoxIterationNum,12,2)[0]
        
        ListCorners = BoBox.GetCorners()
        Pts =  rs.AddPoints(ListCorners)
        if rs.Distance(Pts[0], Pts[1])> rs.Distance(Pts[0], Pts[3]):
            DirVec = rs.VectorCreate(Pts[1], Pts[0])
            Angle = rs.VectorAngle(-DirVec, (1,0,0))
        else:
            DirVec = rs.VectorCreate(Pts[3], Pts[0])
            Angle = rs.VectorAngle(-DirVec, (1,0,0))
        #print Angle
        
        
        #Documenting To Excel
        #print GetPlanOnBrep
        BlockLayer = rs.ObjectLayer(i)
        #Get The Block Color
        BlockColor = rs.LayerColor(BlockLayer)
        #Writing Values To Excel
        ws.Cells[2+index, 7].Interior.Color = BlockColor
        #WritingIndex
        ws.Cells[2+index, 1].Value2 = index
        #WritingBlockName
        ws.Cells[2+index, 2].Value2 = BlName
        #WritingBlockXValue
        ws.Cells[2+index, 3].Value2 = PtX
        #WritingBlockYValue
        ws.Cells[2+index, 4].Value2 = PtY
        #WritingBlockZValue
        ws.Cells[2+index, 5].Value2 = PtZ
        #WritingBlockRotationValue
        ws.Cells[2+index, 6].Value2 = Angle
        #DeleteGeneratedGeometries
        rs.DeleteObject(Bl01)
        rs.DeleteObjects(Pts)
    
#======================Get Rhino Objects===========================#

#===========================Methods================================#



