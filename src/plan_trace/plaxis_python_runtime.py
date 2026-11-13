# Rconcrete FEM -> PLAXIS 3D Input Remote Scripting.
# Start the PLAXIS Input scripting server. Run with PLAXIS Python (plxscripting).
# Work in a COPY: importing modifies geometry and is not transactional.
# Credentials: PLAXIS_HOST, PLAXIS_PORT, PLAXIS_PASSWORD, or interactive prompts.
import os, re, json, math, getpass, sys, traceback
from pathlib import Path

def _pause():
    if os.environ.get('PLAXIS_NO_PAUSE') != '1' and sys.stdin is not None and sys.stdin.isatty():
        try: input('Press Enter to close...')
        except (EOFError, KeyboardInterrupt): pass

def _error_hook(kind, error, tb):
    message = ''.join(traceback.format_exception(kind, error, tb))
    print(message, file=sys.stderr)
    log = Path(__file__).with_suffix('.error.log')
    try:
        log.write_text('Python: '+sys.executable+'\n'+message, encoding='utf-8')
        print('Error log: '+str(log), file=sys.stderr)
    except OSError as failure:
        print('Could not write error log: '+str(failure), file=sys.stderr)
    _pause()

sys.excepthook = _error_hook
print('Rconcrete FEM -> PLAXIS 3D')
print('Python: '+sys.executable)
try:
    from plxscripting.easy import new_server
except ImportError as error:
    raise RuntimeError('PLAXIS scripting module is unavailable in this Python. '
        'Run the companion CMD launcher or use the Python supplied with PLAXIS. '
        'Original error: '+str(error)) from error
host = os.environ.get('PLAXIS_HOST', 'localhost')
port = os.environ.get('PLAXIS_PORT')
if not port:
    port = input('PLAXIS Input server port [10000]: ').strip() if sys.stdin is not None and sys.stdin.isatty() else ''
port = int(port or '10000')
print('Connecting to PLAXIS Input at %s:%s...'%(host,port))

def _server_password():
    password = os.environ.get('PLAXIS_PASSWORD') or getpass.getpass('PLAXIS scripting password: ')
    if not 4 <= len(password.encode('utf-8')) <= 56:
        raise RuntimeError('PLAXIS password must encode to 4..56 UTF-8 bytes for Blowfish. '
            'Set an 8-16 character Latin-letter/digit password in the PLAXIS Input server, '
            'then enter the SAME password here. The script does not change or pad passwords.')
    return password

s_i, g_i = new_server(host, port,
    password=_server_password())

def _value(x):
    return getattr(x, 'value', x)

def _name(x):
    return str(_value(x.Name))

def _ref(token):
    obj = g_i
    for part in token.split('.'):
        m = re.fullmatch(r'([A-Za-z_][A-Za-z_0-9]*)(?:\[(-?\d+)\])?', part)
        if not m: raise RuntimeError('Unsupported generated object reference: '+token)
        obj = getattr(obj, m[1])
        if m[2] is not None: obj = obj[int(m[2])]
    return obj

def _exec(line):
    tokens = re.findall(r'"(?:\\.|[^"\\])*"|\S+', line)
    args = []
    for t in tokens[1:]:
        if t.startswith('"'): args.append(json.loads(t))
        else:
            try: args.append(float(t))
            except ValueError: args.append(_ref(t))
    return getattr(g_i, tokens[0].lstrip('_'))(*args)

def _area(points):
    return abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(points,points[1:]+points[:1])))/2

def _inside(p, ring):
    hit = False
    for a,b in zip(ring,ring[1:]+ring[:1]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        t=((p[0]-a[0])*dx+(p[1]-a[1])*dy)/max(dx*dx+dy*dy,1e-30)
        if -1e-8<=t<=1+1e-8 and abs(dx*(p[1]-a[1])-dy*(p[0]-a[0]))<=1e-8*max(1,math.hypot(dx,dy)): return True
        if (a[1]>p[1])!=(b[1]>p[1]) and p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]: hit=not hit
    return hit

def _vertices(obj):
    # API geometry exposure varies between versions. No guessed bounding-box deletion.
    for prop in ('Points', 'Vertices', 'Coordinates'):
        try:
            raw=list(getattr(obj,prop))
            pts=[]
            for p in raw:
                if isinstance(p,(tuple,list)) and len(p)>=3: pts.append([float(_value(v)) for v in p[:3]])
                else: pts.append([float(_value(getattr(p,a))) for a in ('x','y','z')])
            if len(pts)>=3: return pts
        except (AttributeError,TypeError,ValueError): pass
    raise RuntimeError('Cannot verify boundary of '+_name(obj)+'. No geometry deleted; this PLAXIS API needs a boundary adapter.')

def _surface_area(obj):
    try: return float(_value(obj.Area))
    except (AttributeError,TypeError,ValueError):
        raise RuntimeError('Cannot verify Area of '+_name(obj)+'. No geometry deleted.')

def _surfaces():
    collection=g_i.Surfaces
    items=collection[:]
    # Some server versions return null for an empty remote slice.
    # Verify the count before treating null as empty, so an API failure
    # cannot hide existing geometry from the ownership checks.
    if items is None:
        count=len(collection)
        if count!=0:
            raise RuntimeError('PLAXIS Surfaces returned no items but reports %s surfaces; stopped before geometry changes'%count)
        return {}
    return {_name(o):o for o in items}

def _make_surface(points):
    # PLAXIS defines the plane from the first three vertices.
    for i in range(len(points)):
        a,b,c=points[i],points[(i+1)%len(points)],points[(i+2)%len(points)]
        u=[b[j]-a[j] for j in range(3)];v=[c[j]-a[j] for j in range(3)]
        if sum(x*x for x in (u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]))>1e-20:
            points=points[i:]+points[:i];break
    else: raise RuntimeError('Degenerate surface; no geometry created')
    before=_surfaces()
    result=g_i.surface(*[tuple(p) for p in points])
    after=_surfaces()
    added=[o for n,o in after.items() if n not in before]
    if len(added)!=1: raise RuntimeError('Surface creation did not yield exactly one identifiable surface; stopped.')
    return added[0]

def _slab(source, outer, holes, assign):
    if any(n.startswith('RCFSlab%sPart'%source) for n in _surfaces()):
        raise RuntimeError('Slab %s already imported; use a fresh model copy'%source)
    material = g_i.Materials[-1] if assign else None
    retained=[_make_surface(outer)]
    expected=_area(outer)
    for hole_index,ring in enumerate(holes,1):
        # Reject topologies that need a general split classifier instead of guessing.
        if not all(_inside(p,outer) for p in ring): raise RuntimeError('Slab %s: hole is not contained in parent'%source)
        cutter=_make_surface(ring)
        before=_surfaces()
        protected=set(before)-{_name(o) for o in retained+[cutter]}
        inputs=retained+[cutter]
        result=g_i.intersect(*inputs)
        # Command return references are the ownership boundary, not global last-object indices.
        flat=list(result) if isinstance(result,(tuple,list)) else [result]
        returned={}
        for obj in flat:
            try: returned[_name(obj)]=obj
            except (AttributeError,TypeError): pass
        after=_surfaces()
        candidates=[obj for name,obj in after.items() if name in returned]
        if any(_name(o) in protected for o in candidates): raise RuntimeError('Intersect returned unrelated geometry; stopped before deletion')
        if not candidates: raise RuntimeError('Slab %s: intersect did not return identifiable surface references; stopped before deletion'%source)
        hole_area=_area(ring)
        tol=max(1.,expected)*1e-7
        new_expected=expected-hole_area
        records=[]
        for obj in candidates:
            area=_surface_area(obj)
            try: vertices=_vertices(obj)
            except RuntimeError: vertices=None
            inside=None if vertices is None else all(
                abs(p[2]-ring[0][2])<1e-7 and _inside(p,ring) for p in vertices)
            records.append((obj,area,inside,vertices))
        interior=[obj for obj,area,inside,vertices in records if inside is True]
        exterior=[obj for obj,area,inside,vertices in records if inside is not True]
        def balanced(inner,outer_parts):
            return (inner and outer_parts
                and abs(sum(_surface_area(o) for o in inner)-hole_area)<=tol
                and abs(sum(_surface_area(o) for o in outer_parts)-new_expected)<=tol)
        if not balanced(interior,exterior):
            # For a contained cut, a unique returned part with exactly the cutter
            # area plus conservation of the remaining plate identifies the hole
            # even when PLAXIS does not expose that part's boundary. Never override
            # an available boundary that places a part outside the cutter.
            matches=[obj for obj,area,inside,vertices in records
                if inside is not False and abs(area-hole_area)<=tol]
            if len(matches)==1:
                inner_name=_name(matches[0])
                rest=[obj for obj in candidates if _name(obj)!=inner_name]
                if balanced(matches,rest): interior,exterior=matches,rest
        if not balanced(interior,exterior):
            details={'slab':source,'hole':hole_index,'plate_area':expected,
                'hole_area':hole_area,'expected_remaining':new_expected,'tolerance':tol,
                'returned':[{'name':_name(obj),'area':area,'inside_hole':inside,
                    'vertex_count':None if vertices is None else len(vertices),
                    'vertex_sample':None if vertices is None else vertices[:8]}
                    for obj,area,inside,vertices in records]}
            raise RuntimeError('Cannot uniquely verify interior fragments; stopped before deletion. '
                +json.dumps(details,ensure_ascii=False))
        for obj in interior: g_i.delete(obj)
        retained=exterior; expected=new_expected
    for index,obj in enumerate(retained):
        g_i.rename(obj,'RCFSlab%sPart%s'%(source,index+1))
        if material is not None:
            g_i.plate(obj)
            g_i.set(obj.Plate.Material,material)
    print('Slab',source,'surfaces',len(retained),'area',expected)

def _check_units():
    # Enumeration.value is a numeric code; strvalue is the API's symbolic name.
    for field, expected_unit in [('UnitLength','m'),('UnitForce','kN')]:
        prop=getattr(g_i.Project,field)
        actual=getattr(prop,'strvalue',None)
        if actual is None:
            raw=_value(prop)
            if isinstance(raw,str): actual=raw
            else:
                choices=getattr(prop,'enum_dict',{})
                actual=next((name for name,code in choices.items() if code==raw),None)
        if actual is None:
            raise RuntimeError('Cannot read symbolic PLAXIS '+field+'; numeric unit codes are not guessed.')
        if str(actual).strip().casefold()!=expected_unit.casefold():
            raise RuntimeError('PLAXIS '+field+' must be '+expected_unit+', got '+str(actual))

# Validate units before creating anything. Never change an existing model's units silently.
_check_units()
