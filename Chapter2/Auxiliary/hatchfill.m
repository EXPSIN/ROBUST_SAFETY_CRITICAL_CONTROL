function H = hatchfill(A,STYL,ANGLE,SPACING,FACECOL)

if nargin == 1
    STYL = 'single';
    ANGLE = 45;
    SPACING = 5;
    FACECOL = 'none';
end

if strcmpi(STYL,'none')
    STYL = 'fill';
end

if nargin == 2
    if strcmpi(STYL,'single') || strcmpi(STYL,'cross')
        ANGLE = 45;
        SPACING = 5;
        FACECOL = 'none';
    elseif strcmpi(STYL,'speckle') || strcmpi(STYL,'outspeckle')
        ANGLE = 7;
        SPACING = 1;
        FACECOL = 'none';
    elseif strcmpi(STYL,'fill')
        FACECOL = [0.8 0.8 0.8];
    end
end

if nargin == 3
    error('Invalid number of input arguments');
end

if nargin == 4
    if strcmpi(STYL,'fill')
        FACECOL = [0.8 0.8 0.8];
    else
        FACECOL = 'none';
    end
end

if ( ~strcmpi(STYL,'single') && ~strcmpi(STYL,'cross') && ...
     ~strcmpi(STYL,'speckle') && ~strcmpi(STYL,'outspeckle') && ...
     ~strcmpi(STYL,'fill') )
    error(['Invalid style: ',STYL])
end

linec = 'k';
linew = 0.5;
specksize = 2;

hax = get(A(1),'parent');
is_axes = strcmpi(get(hax,'type'),'axes');
if ~is_axes
   hax = get(hax,'parent');
end
is_axes = strcmpi(get(hax,'type'),'axes');

x_is_log = 0; y_is_log = 0;
x_is_reverse = 0; y_is_reverse = 0;

if is_axes
   axsize_in = get(hax,'position');
   y_is_log = strcmpi(get(hax,'yscale'),'log');
   if y_is_log
       ylims = get(hax,'ylim');
       dy = (ylims(2) - ylims(1))/(log10(ylims(2))-log10(ylims(1)));
       set(hax,'units','pixels');
       axsize = get(hax,'position');
       set(hax,'position',[ axsize(1:3) dy*axsize(4) ]);
       set(hax,'units','normalized')
   end

   x_is_log = strcmpi(get(hax,'xscale'),'log');
   if x_is_log
       xlims = get(hax,'xlim');
       dx = (xlims(2) - xlims(1))/(log10(xlims(2))-log10(xlims(1)));
       set(hax,'units','pixels');
       axsize = get(hax,'position');
       set(hax,'position',[ axsize(1:2) dx*axsize(3) axsize(4) ]);
       set(hax,'units','normalized')
   end

   if strcmp(STYL,'single') || strcmp(STYL,'cross')
      y_is_reverse = strcmpi(get(hax,'ydir'),'reverse');
      if y_is_reverse
          ANGLE = -ANGLE;
      end
      x_is_reverse = strcmpi(get(hax,'xdir'),'reverse');
      if x_is_reverse
          ANGLE = 180-ANGLE;
      end
   end
end

j = 1;
for k = 1:length(A)
    set(A,'facecolor',FACECOL);
    v = get(A(k),'vertices');
    if any(v(end,:)~=v(1,:))
        v(end+1,:) = v(1,:);
    end
    x = v(:,1);
    if x_is_log
        x = log10(v(:,1));
    end
    y = v(:,2);
    if y_is_log
        y = log10(v(:,2));
    end

    if strcmp(STYL,'fill')
        H = NaN;
        continue
    end

    [xhatch,yhatch] = hatch_xy(x,y,STYL,ANGLE,SPACING);
    if x_is_log
        xhatch = 10.^xhatch;
    end
    if y_is_log
        yhatch = 10.^yhatch;
    end
    if strcmp(STYL,'speckle') || strcmp(STYL,'outspeckle')
        if any(xhatch)
            H(j) = line(xhatch,yhatch,'marker','.','linest','none', ...
                    'markersize',specksize,'color',linec);
            j = j+1;
        end
    elseif strcmp(STYL,'single') || strcmp(STYL,'cross')
        H(j) = line(xhatch,yhatch);
        set(H(j),'color',linec,'linewidth',linew);
        j = j+1;
    end
end

if y_is_log || x_is_log
    set(hax,'position',axsize_in);
end






function [xi,yi,x,y]=hatch_xy(x,y,varargin);


styl='speckle';
angle=7;
step=1/2;

if length(varargin)>0 & isstr(varargin{1}),
  styl=varargin{1};
  varargin(1)=[];
end;
if length(varargin)>0 & ~isstr(varargin{1}),
  angle=varargin{1};
  varargin(1)=[];
end;
if length(varargin)>0 & ~isstr(varargin{1}),
  step=varargin{1};
  varargin(1)=[];
end;

I = zeros(1,length(x));


if x(end)~=x(1) & y(end)~=y(1),
  x=x([1:end 1]);
  y=y([1:end 1]);
  I=I([1:end 1]);
end;

if strcmp(styl,'speckle') | strcmp(styl,'outspeckle'),
  angle=angle*(1-I);
end;

if size(x,1)~=1,
 x=x(:)';
 angle=angle(:)';
end;
if size(y,1)~=1,
 y=y(:)';
end;


oldu = get(gca,'units');
set(gca,'units','points');
sza = get(gca,'pos'); sza = sza(3:4);
set(gca,'units',oldu)

xlim = get(gca,'xlim');
ylim = get(gca,'ylim');
xsc = sza(1)/(xlim(2)-xlim(1)+eps);
ysc = sza(2)/(ylim(2)-ylim(1)+eps);

switch lower(styl),
 case 'single',
  [xi,yi]=drawhatch(x,y,angle,step,xsc,ysc,0);
  if nargout<2,
    xi=line(xi,yi,varargin{:});
  end;
 case 'cross',
  [xi,yi]=drawhatch(x,y,angle,step,xsc,ysc,0);
  [xi2,yi2]=drawhatch(x,y,angle+90,step,xsc,ysc,0);
  xi=[xi,xi2];
  yi=[yi,yi2];
  if nargout<2,
    xi=line(xi,yi,varargin{:});
  end;
 case 'speckle',
  [xi,yi ]  =drawhatch(x,y,45,   step,xsc,ysc,angle);
  [xi2,yi2 ]=drawhatch(x,y,45+90,step,xsc,ysc,angle);
  xi=[xi,xi2];
  yi=[yi,yi2];
  if nargout<2,
    if any(xi),
      xi=line(xi,yi,'marker','.','linest','none','markersize',2,varargin{:});
    else
      xi=NaN;
    end;
  end;
 case 'outspeckle',
  [xi,yi ]  =drawhatch(x,y,45,   step,xsc,ysc,-angle);
  [xi2,yi2 ]=drawhatch(x,y,45+90,step,xsc,ysc,-angle);
  xi=[xi,xi2];
  yi=[yi,yi2];
  inside=logical(inpolygon(xi,yi,x,y));
  xi(inside)=[];yi(inside)=[];
  if nargout<2,
    if any(xi),
      xi=line(xi,yi,'marker','.','linest','none','markersize',2,varargin{:});
    else
      xi=NaN;
    end;
  end;

end;


return


function [xi,yi]=drawhatch(x,y,angle,step,xsc,ysc,speckle);

angle=angle*pi/180;

ca = cos(angle); sa = sin(angle);
x0 = mean(x); y0 = mean(y);
x = (x-x0)*xsc; y = (y-y0)*ysc;
yi = x*ca+y*sa;
y = -x*sa+y*ca;
x = yi;
y = y/step;

yi = ceil(y);
yd = [diff(yi) 0];
fnd = find(yd);
dm = max(abs(yd));


A = cumsum( repmat(sign(yd(fnd)),dm,1), 1);

fnd1 = find(abs(A)<=abs( repmat(yd(fnd),dm,1) ));
A  = A+repmat(yi(fnd),dm,1)-(A>0);
xy = (x(fnd+1)-x(fnd))./(y(fnd+1)-y(fnd));
xi = repmat(x(fnd),dm,1)+(A-repmat(y(fnd),dm,1) ).*repmat(xy,dm,1);
yi = A(fnd1);
xi = xi(fnd1);


xi0 = min(xi); xi1 = max(xi);
ci = 2*yi*(xi1-xi0)+xi;
[ci,num] = sort(ci);
xi = xi(num); yi = yi(num);


if rem(length(xi),2)==1,
  disp('mhatch warning');
  xi = [xi; xi(end)];
  yi = [yi; yi(end)];
end

li = length(xi);
xi = reshape(xi,2,li/2);
yi = reshape(yi,2,li/2);

if length(speckle)>1 | speckle(1)~=0,

 if length(speckle)>1,

   yd=[speckle(1:end)];
   A=repmat(yd(fnd),dm,1);
   speckle=A(fnd1);

   speckle=speckle(num);
   if rem(length(speckle),2)==1,
     speckle = [speckle; speckle(end)];
   end
   speckle=reshape(speckle,2,li/2);

 else
   speckle=[speckle;speckle];
 end;

 oldxi=xi;oldyi=yi;
 dxi=diff(xi);
 nottoosmall=sum(speckle,1)~=0 & rand(1,li/2)<abs(dxi)./(max(sum(speckle,1),eps));
 xi=xi(:,nottoosmall);
 yi=yi(:,nottoosmall);
 dxi=dxi(nottoosmall);
 if size(speckle,2)>1, speckle=speckle(:,nottoosmall); end;
 li=length(dxi);
 if any(li),
   xi(1,:)=xi(1,:)+sign(dxi).*(1-rand(1,li).^0.5).*min(speckle(1,:),abs(dxi) );
   xi(2,:)=xi(2,:)-sign(dxi).*(1-rand(1,li).^0.5).*min(speckle(2,:),abs(dxi) );
   if size(speckle,2)>1,
    xi=xi(speckle~=0);
    yi=yi(speckle~=0);
   end;
  end;

else
 xi = [xi; ones(1,li/2)*nan];
 yi = [yi; ones(1,li/2)*nan];
end;
xi = xi(:)'; yi = yi(:)';

yi = yi*step;
xy = xi*ca-yi*sa;
yi = xi*sa+yi*ca;
xi = xy/xsc+x0;
yi = yi/ysc+y0;
