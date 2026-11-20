"""Generate original millimetre cut profiles and dimensioned assembly drawings."""
from pathlib import Path
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Rectangle,Circle

OUT=Path(__file__).parent/'drawings'
HEIGHT=127.
SIDE=846.2245
STOCK=19.05
PANELS={
    'Front':(711.2,[120.6,590.6]), 'Rear':(711.2,[180.6,530.6]),
    'Left':(SIDE,[147.512,648.512]), 'Right':(SIDE,[197.7125,698.7125])}


def outline(w,h):
    return [(0,0),(w,0),(w,h),(0,h)]


def cut_dxf(path,w,h,bores,pockets):
    pairs=[]
    def add(*entries):
        pairs.extend(entries)
    def loop(layer,points):
        for a,b in zip(points,points[1:]+points[:1]):
            add(0,'LINE',8,layer,10,a[0],20,a[1],30,0,11,b[0],21,b[1],31,0)
    add(0,'SECTION',2,'HEADER',9,'$ACADVER',1,'AC1015',9,'$INSUNITS',70,4,0,'ENDSEC',
        0,'SECTION',2,'TABLES',0,'TABLE',2,'LAYER',70,3)
    for layer,color in [('PROFILE',7),('THROUGH',3),('POCKET_1P5',1)]:
        add(0,'LAYER',2,layer,70,0,62,color,6,'CONTINUOUS')
    add(0,'ENDTAB',0,'ENDSEC',0,'SECTION',2,'ENTITIES')
    loop('PROFILE',outline(w,h))
    for x,y,d in bores:add(0,'CIRCLE',8,'THROUGH',10,x,20,y,30,0,40,d/2)
    for x,y in pockets:loop('POCKET_1P5',[(x-7.6,y-7.6),(x+7.6,y-7.6),(x+7.6,y+7.6),(x-7.6,y+7.6)])
    add(0,'ENDSEC',0,'EOF')
    path.write_text('\n'.join(map(str,pairs))+'\n',encoding='ascii',newline='\n')


def sheet(pdf,title,subtitle,panels,notes):
    fig=plt.figure(figsize=(11.69,8.27))
    fig.text(.065,.92,title,fontsize=20,fontweight='bold',color='#173952')
    fig.text(.066,.875,subtitle,fontsize=10)
    if not panels:
        ax=fig.add_axes([.10,.40,.74,.38])
        ax.set_aspect('equal');ax.axis('off')
        ax.set_xlim(-395,395);ax.set_ylim(-35,147)
        for z in (0,60):
            ax.add_patch(Rectangle((-355.6,z),711.2,38.1,edgecolor='#173952',facecolor='#dbe6ed'))
        for y in (-342.9,-160,160,342.9):
            ax.add_patch(Rectangle((y-12.7,38.1),25.4,21.9,edgecolor='#173952',facecolor='#e4ad6a'))
        ax.text(0,113,'Outer bumper rail     x = 454.462',ha='center',fontsize=10)
        ax.text(0,-22,'Inner crossmember     x = 394.462 at forward face',ha='center',fontsize=10)
    h=.46/max(1,len(panels))
    for i,(label,w,height,bores,pockets) in enumerate(panels):
        ax=fig.add_axes([.1+.43*i,.38,.36,.43]) if title.startswith('Upper lift') else fig.add_axes([.09,.78-(i+1)*h,.79,h*.8])
        ax.set_aspect('equal');ax.set_xlim(-w*.03,w*1.03);ax.set_ylim(-height*.17,height*1.25);ax.axis('off')
        ax.add_patch(Rectangle((0,0),w,height,fill=False,linewidth=1.3))
        for x,y,d in bores:
            ax.add_patch(Circle((x,y),d/2,fill=False,linewidth=.9))
            ax.plot([x-8,x+8],[y,y],color='#367782',linewidth=.4)
        for x,y in pockets:ax.add_patch(Rectangle((x-7.6,y-7.6),15.2,15.2,fill=False,color='#9d421f',linewidth=.8))
        ax.text(0,height*1.08,label,fontsize=9)
    for i,line in enumerate(notes):fig.text(.07,.32-i*.041,line,fontsize=9)
    fig.text(.07,.035,'2024 student robot  /  R9 parked assembly     •     dimensions in mm     •     PDF views schematic; DXF 1:1',fontsize=8)
    pdf.savefig(fig);plt.close(fig)


def main():
    OUT.mkdir(exist_ok=True)
    with PdfPages(OUT/'r9-shop-drawings.pdf',metadata={'CreationDate':None,'ModDate':None}) as pdf:
        for end in [('Front','Rear'),('Left','Right')]:
            panels=[]
            for side in end:
                w,holes=PANELS[side]
                centers=[(x,86.5) for x in holes];bores=[(x,y,9.2) for x,y in centers]
                cut_dxf(OUT/(side.lower()+'-board.dxf'),w,HEIGHT,bores,centers)
                panels.append((f'{side}    {w:.4f} × 127 × 19.05    |    hole X = {holes[0]:.4f}, {holes[1]:.4f}',w,HEIGHT,bores,centers))
            sheet(pdf,'Bumper backing panels','Outside face. X is measured to the right when the panel is viewed from outside the robot.',panels,[
                'Origin is the lower-left corner of each drawn blank. Both hole centers are 86.5 from the lower edge.',
                'Two Ø9.20 through-holes. Two square nut pockets, 15.20 × 15.20, 1.50 deep from the outside face.',
                'Captive barrel nut: 15 × 15 × 1.50 flange, Ø9.00 barrel, 12.00 overall, M6 × 1 internal thread.',
                'Target panel length ±0.50, hole centers ±0.15 and pocket depth ±0.10. Inspect actual stock before routing.',
                'Seat flange flush. Trial fit on completed rails. Install foam and cloth after all eight fasteners pass.',
                'Panel names follow robot +X forward and +Y left. Confirm orientation against the two asymmetric side stations.'])
        w,h=75.4,100.
        c=(46.5,44.5)
        r=25.4/math.sqrt(2)
        bores=[(*c,32)]+[(c[0]+dx,c[1]+dz,5.1) for dx,dz in [(-r,-r),(r,r),(-r,r),(r,-r)]]
        cut_dxf(OUT/'lift-gearbox-tab.dxf',w,h,bores,[])
        ob=[(21.,21.,28.605)]
        cut_dxf(OUT/'lift-bearing-tab.dxf',49.9,76.5,ob,[])
        sheet(pdf,'Upper lift mounting plates','Two of each plate, 6.35 thick 6061-T6. Origin is the lower-rear corner looking along the shaft.',[
            ('Gearbox tab   75.40 × 100.00',w,h,bores,[]),('Bearing tab   49.90 × 76.50',49.9,76.5,ob,[])],[
            'Gearbox bore Ø32 at (46.50, 44.50); four Ø5.10 clearance holes on Ø50.80 PCD, clocked 45°.',
            'Bearing seat Ø28.605 nominal at (21.00, 21.00). Inspect purchased bearing OD before final boring.',
            'Gearbox inside faces y = ±25.40; bearing plates span |y| = 274.15–280.50. Weld edges seat against x = 88.90.',
            'Jig the Ø13.75 rounded shaft axis at x = 60.00, z = 898.50 before tacking. Recheck alignment after cooling.',
            'Shaft cut length 595.50. Finish 10-32 ends for the end screws; inspect thread depth before assembly.',
            'The weld joint and bearing life need structural qualification. Shaft deflection and thermal growth need a bench check.'])
        sheet(pdf,'Frame and mast attachment','Front crossmembers run along +Y. The four weld spacers run fore-aft.',[],[
            'Front member: 711.20 long, 38.10 × 50.80 × 3.175 tube; outer face x = 454.462. Two inherited M6 landings at y = ±235.',
            'Inner member: same stock and length, outer face x = 394.462. Four spacers bridge 21.90 between weld faces.',
            'Spacer stock: 25.4 × 50.8 × 3.175; center y = ±342.9 and ±160.0. Fixture on a common lower face z = 74.60.',
            'Rear face x = −353.6625. Target overall length 808.1245 ±1.0 and width 711.20 ±1.0 after welding.',
            'Four mast stations x = 23, 53 and y = ±329. Drill both tube walls Ø13.10, insert Ø13.00 / Ø8.50 sleeves, weld and face flush.',
            'Sleeve cut lengths 50.80 and 38.35. Insert sleeves while the two loose tubes are accessible. Deburr before joining.',
            'Fit M8 × 100 cap screw from above; nominal stack closes with an under-rail washer and class 8 M8 nut.'])
        from breaker_layout import BASE_HOLES, POSTS
        xy_local=lambda x,y:(x+320,y+310)
        details=[(*xy_local(x,y),d) for group,d in [(BASE_HOLES,5.5),(sum(POSTS,()),4.5)] for x,y in group]
        # Detail includes only the rework zone and datums, never the full deck perimeter.
        sheet(pdf,'Breaker and cable-support deck rework','Top view. Robot +X forward, +Y left. Existing deck lower face z = 202.115.',[
            ('Rework detail from deck corner (−320, −310)',155,245,details,[])],[
            'Locate from the straight deck edges x = −320 and y = −310. Preserve the four previous pedestal holes.',
            'Pedestal: Ø5.50 through at x = −306 and −244, y = −168 and −146. Jig with two diagonal clearance pins; remove after transfer.',
            'Stand-offs: Ø4.50 through at (−290, −130), (−312, −88.5), (−222, −155), (−199, −137). Break both edges.',
            'Four 6061 posts Ø10 × 43.710, M4 × 0.7 both ends, 12 tap-drill depth. Check at least 7.0 usable thread at each end.',
            'M4 × 10 deck and shelf screws enter posts 7.000 and 6.825 respectively before thread starts. Verify usable engagement.',
            'The 3.175 thick shelf and 10 high lower saddle form one machined piece. Fabricate against the selected assembly.'])
        fig,axes=plt.subplots(1,2,figsize=(11.69,8.27))
        fig.subplots_adjust(top=.79,bottom=.41,left=.075,right=.94,wspace=.31)
        fig.text(.065,.92,'Local cable clamp and saddle',fontsize=20,fontweight='bold',color='#173952')
        fig.text(.066,.875,'Section across lead at u = 45 mm   /   cap and saddle section assembled with 0.30 mm shims',fontsize=10)
        ax=axes[0]
        for y0,y1 in [(-10,-.15),(.15,10)]:
            ax.add_patch(Rectangle((-17,y0),34,y1-y0,facecolor='#dbe6ed',edgecolor='#173952'))
        for v in (-11.5,11.5):
            ax.add_patch(Rectangle((v-4.5,-.15),9,.30,facecolor='#b3763b'))
            ax.plot([v,v],[-11,12],color='#276982',linestyle='--',linewidth=.8)
        ax.add_patch(Circle((0,0),5.5,facecolor='#434d50'))
        ax.add_patch(Circle((0,0),3.875,facecolor='#b64236'))
        ax.set(xlim=(-21,21),ylim=(-14,14),title='Clamp section, local v and z (mm)',xlabel='Across cable')
        ax.set_aspect('equal');ax.grid(alpha=.15)
        ax=axes[1]
        for index,(pair,color) in enumerate(zip(POSTS,('#286478','#914f2b')),1):
            from breaker_layout import PLATFORMS,ROUTES,xy
            x0,y0,x1,y1=PLATFORMS[index-1]
            ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,edgecolor=color))
            for x,y in pair:ax.add_patch(Circle((x,y),2.25,fill=False,edgecolor=color))
            verts=[xy(ROUTES[index-1],u,v) for u,v in ((38,-17),(52,-17),(52,17),(38,17),(38,-17))]
            ax.plot(*zip(*verts),color=color,linewidth=1)
        ax.set(xlim=(-350,-178),ylim=(-192,-75),xlabel='Robot X (mm)',ylabel='Robot Y (mm)',title='Shelf outlines and saddle locations')
        ax.set_aspect('equal');ax.grid(alpha=.15)
        notes=['Make two 6061 caps 14 long × 34 wide. Cap and lower jaw height 9.85 each. Align with the actual local lead azimuth.',
               'Fixture with 0.30 mm shims at both screws, then finish the concentric Ø11 bore. Cap Ø4.50; tap lower M4 × 0.7 at v = ±11.50.',
               'Each replaceable shim is 14 × 9 × 0.30, through Ø4.50 centered. Nominal screw underside z = 272; gross lower entry 5.85.',
               'Machine the 3.175 mm shelf and lower jaw together. Shelf 1 limits x = −340…−284, y = −135…−84; shelf 2 x = −230…−185, y = −183…−131.',
               'EPDM nominal free thickness 1.80. The Ø7.75 cable compresses nominal radial thickness to 1.625. Adjust both shims together.',
               'Measure the received jacket, inspect the sheath after a bench pull, and qualify grip before connecting the harness.']
        for i,n in enumerate(notes):fig.text(.065,.33-i*.043,n,fontsize=8.7)
        fig.text(.065,.035,'2024 student robot  /  R9 parked assembly     •     dimensions in mm',fontsize=8)
        pdf.savefig(fig);plt.close(fig)
if __name__=='__main__':main()
