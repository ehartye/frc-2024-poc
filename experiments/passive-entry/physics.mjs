/** A free NOTE with torus centerline quadrature, carpet contact, and two passive tread rails.
 * Positions in mm, forces in N, clock in s. No lateral servo or attachment constraint. */
import {readFileSync} from 'node:fs';
export const design=JSON.parse(readFileSync(new URL('part.json',import.meta.url)));
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const mag=a=>Math.hypot(...a);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const round=(x,n=4)=>Number(x.toFixed(n));

export function simulate(frames,initial,options={}){
  const d=design, radius=d.railRadiusMm, tube=d.tubeMm, R=d.majorMm;
  const steps=options.steps??50, dt=.05/steps, N=options.grid??96, mass=d.massKg;
  const ds=2*Math.PI*R/N, stiffness=options.stiffness??d.railStiffnessNPerMm2;
  const carpet=options.carpet??d.floorFriction, friction=options.friction??d.railFriction;
  const path=d.railPathMm.map(([x,y])=>[x,y+(options.widthError??0)]);
  const zz=d.railZmm+(options.heightError??0);
  const positions=Array.from({length:N},(_,i)=>[R*Math.cos(2*Math.PI*i/N),R*Math.sin(2*Math.PI*i/N)]);
  const p=[...initial,tube], velocity=[0,0,0], omega=[0,0,0], q=[1,0,0,0];
  const inertia=[mass*(R*R/2+5*tube*tube/8), mass*(R*R+3*tube*tube/4)];
  const springs=new Float64Array(6*N);
  const trace=[],phaseStats={};
  let maxDepth=0, minCore=Infinity, maxTilt=0, lastRail=-Infinity, maxTop=-Infinity, firstContact=null;
  const last=frames.length-1;
  for(let j=0;j<=last;j++){
    const [time,X,Y,Z,phase]=frames[j], next=frames[Math.min(last,j+1)];
    const railV=[(next[1]-X)/.05,(next[2]-Y)/.05,(next[3]-Z)/.05];
    const stats=phaseStats[phase]??={minBottom:Infinity,maxBottom:-Infinity,maxLateral:0,maxXDeviation:0,maxFloor:0,maxRail:0,minRail:Infinity,minSide:Infinity,maxTop:-Infinity,minX:Infinity,maxX:-Infinity};
    for(let sub=0;sub<(j===last?1:steps);sub++){
      const frac=sub/steps, cx=X+frac*(next[1]-X), cy=Y+frac*(next[2]-Y), cz=Z+frac*(next[3]-Z);
      const [w,x,y,z]=q;
      const u=[1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)];
      const v=[2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x)];
      const axis=cross(u,v);
      const tilt=Math.hypot(u[2],v[2]), bottom=p[2]-tube-R*tilt,top=p[2]+tube+R*tilt;
      const F=[0,0,-mass*9.80665], M=[0,0,0], sides=[0,0];
      let floor=0;
      // Each torus strip has a persistent tangential spring for carpet and tread.
      // Project it into the current tangent plane, then apply a Coulomb return map.
      function contact(i,slot,nx,ny,nz,depth,ax,ay,az,k,mu,svx,svy,svz){
        const offset=(i*2+slot)*3;
        if(depth<=0){springs.fill(0,offset,offset+3);return 0;}
        const rx=ax-tube*nx,ry=ay-tube*ny,rz=az-tube*nz;
        const vx=velocity[0]+omega[1]*rz-omega[2]*ry-svx;
        const vy=velocity[1]+omega[2]*rx-omega[0]*rz-svy;
        const vz=velocity[2]+omega[0]*ry-omega[1]*rx-svz;
        const vn=vx*nx+vy*ny+vz*nz;
        const normal=Math.max(0,k*ds*(depth-.006*vn));
        let tx=springs[offset]+dt*(vx-vn*nx),ty=springs[offset+1]+dt*(vy-vn*ny),tz=springs[offset+2]+dt*(vz-vn*nz);
        const off=tx*nx+ty*ny+tz*nz;
        tx-=off*nx;ty-=off*ny;tz-=off*nz;
        const kt=k*ds*.6, size=Math.hypot(tx,ty,tz), cap=mu*normal;
        const scale=size?Math.min(1,cap/(kt*size)):1;
        tx*=scale;ty*=scale;tz*=scale;
        springs[offset]=tx;springs[offset+1]=ty;springs[offset+2]=tz;
        const fx=normal*nx-kt*tx,fy=normal*ny-kt*ty,fz=normal*nz-kt*tz;
        F[0]+=fx;F[1]+=fy;F[2]+=fz;M[0]+=ry*fz-rz*fy;M[1]+=rz*fx-rx*fz;M[2]+=rx*fy-ry*fx;
        return normal;
      }
      for(let i=0;i<N;i++){
        const a=positions[i][0],b=positions[i][1];
        const ax=a*u[0]+b*v[0],ay=a*u[1]+b*v[1],az=a*u[2]+b*v[2];
        floor+=contact(i,0,0,0,1,tube-p[2]-az,ax,ay,az,.035,carpet,0,0,0);
        const px=p[0]+ax-cx,py=p[1]+ay-cy,pz=p[2]+az-cz;
        const side=py<0?-1:1;
        let best=Infinity,dx=0,dy=0,dz=pz-zz;
        if(Math.abs(py)>145 && px>310 && px<540 && Math.abs(dz)<tube+radius){
          for(let k=1;k<path.length;k++){
            const sx=path[k-1][0],sy=side*path[k-1][1],lx=path[k][0]-sx,ly=side*path[k][1]-sy;
            const along=Math.max(0,Math.min(1,((px-sx)*lx+(py-sy)*ly)/(lx*lx+ly*ly)));
            const ex=px-sx-along*lx,ey=py-sy-along*ly,h=ex*ex+ey*ey+dz*dz;
            if(h<best){best=h;dx=ex;dy=ey;}
          }
        }
        const dist=Math.sqrt(best),depth=tube+radius-dist;
        if(depth>0){
          maxDepth=Math.max(depth,maxDepth);minCore=Math.min(minCore,radius-d.coreRadiusMm-depth);
          sides[side<0?0:1]+=contact(i,1,dx/dist,dy/dist,dz/dist,depth,ax,ay,az,stiffness,friction,...railV);
        }else springs.fill(0,i*6+3,i*6+6);
      }
      const rail=sides[0]+sides[1];
      if(rail>.001){lastRail=time+sub*dt;firstContact??=lastRail;}
      maxTilt=Math.max(maxTilt,Math.asin(Math.min(1,tilt)));maxTop=Math.max(maxTop,top);
      stats.minBottom=Math.min(stats.minBottom,bottom); stats.maxBottom=Math.max(stats.maxBottom,bottom);
      stats.maxFloor=Math.max(stats.maxFloor,floor);stats.maxRail=Math.max(stats.maxRail,rail);
      stats.minRail=Math.min(stats.minRail,rail);stats.minSide=Math.min(stats.minSide,...sides);
      stats.maxLateral=Math.max(stats.maxLateral,Math.abs(p[1]-cy));
      stats.minX=Math.min(stats.minX,p[0]-cx);stats.maxX=Math.max(stats.maxX,p[0]-cx);
      stats.maxXDeviation=Math.max(stats.maxXDeviation,Math.abs(p[0]-cx-d.seatXmm));stats.maxTop=Math.max(stats.maxTop,top);
      if(sub===0)trace.push([time,...p.map(n=>round(n,3)),round(bottom,3),round(rail,3),round(floor,3),round(tilt,6)]);
      if(j===last)break;
      for(let k=0;k<3;k++){velocity[k]+=1000*F[k]/mass*dt;p[k]+=velocity[k]*dt;}
      const proj=dot(axis,omega), ang=omega.map((h,k)=>inertia[0]*h+(inertia[1]-inertia[0])*proj*axis[k]);
      const gyro=cross(omega,ang),torque=M.map((h,k)=>1000*h-gyro[k]),normal=dot(torque,axis);
      for(let k=0;k<3;k++)omega[k]+=dt*((torque[k]-normal*axis[k])/inertia[0]+normal*axis[k]/inertia[1]);
      const [wx,wy,wz]=omega;
      q[0]+=.5*dt*(-wx*x-wy*y-wz*z);q[1]+=.5*dt*(wx*w+wy*z-wz*y);
      q[2]+=.5*dt*(-wx*z+wy*w+wz*x);q[3]+=.5*dt*(wx*y-wy*x+wz*w);
      const norm=mag(q);for(let k=0;k<4;k++)q[k]/=norm;
    }
  }
  for(const s of Object.values(phaseStats))for(const k of Object.keys(s))s[k]=round(s[k]);
  return {trace,stats:phaseStats,final:p.map(n=>round(n)),endSpeed:round(mag(velocity)),maxDepth:round(maxDepth),
    coreGap: Number.isFinite(minCore)?round(minCore):null,maxTop:round(maxTop),maxTiltDeg:round(maxTilt*180/Math.PI),
    firstContact:firstContact===null?null:round(firstContact),lastContact:Number.isFinite(lastRail)?round(lastRail):null};
}
