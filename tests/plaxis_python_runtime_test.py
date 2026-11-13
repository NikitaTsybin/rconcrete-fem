import sys, types, unittest
from pathlib import Path
RUNTIME=Path(__file__).resolve().parents[1]/'src/plan_trace/plaxis_python_runtime.py'
class Obj:
    def __init__(self,name,area,points=None):
        self.Name=name; self.Area=area
        if points is not None:self.Points=points
class Fake:
    def __init__(self):
        self.Project=types.SimpleNamespace(UnitLength='m',UnitForce='kN')
        self.Surfaces=[Obj('Unrelated',123)]
        self.Materials=['material'];self.deleted=[];self.calls=0
    def surface(self,*points):
        area=abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(points,points[1:]+points[:1])))/2
        obj=Obj('PolygonNew'+str(len(self.Surfaces)),area,list(points));self.Surfaces.append(obj);return obj
    def intersect(self,*inputs):
        self.calls+=1;hole=inputs[-1];area=sum(o.Area for o in inputs[:-1])-hole.Area
        for obj in inputs:self.Surfaces.remove(obj)
        # Reuse the exterior's name: old name lookup must not imply stale input.
        outer=Obj('Surface_1',area)
        inner=Obj('Polygon_'+str(4-self.calls),hole.Area,hole.Points)
        self.Surfaces.extend([outer,inner]);return [outer,inner]
    def delete(self,obj):self.deleted.append(obj);self.Surfaces.remove(obj)
    def rename(self,obj,name):obj.Name=name
class Tests(unittest.TestCase):
    def runtime(self,g):
        # Execute only helper definitions; never connect to PLAXIS.
        s=RUNTIME.read_text(encoding='utf-8-sig');s=s[s.index('def _value'):s.index('# Validate units')]
        ns=dict(g_i=g,os=__import__('os'),re=__import__('re'),json=__import__('json'),math=__import__('math'))
        exec(compile(s,str(RUNTIME),'exec'),ns);return ns
    def test_two_holes_keep_unrelated_geometry_and_recycled_names(self):
        g=Fake();ns=self.runtime(g)
        ns['_slab'](30,[[0,0,0],[10,0,0],[10,10,0],[0,10,0]],[
            [[1,1,0],[2,1,0],[2,2,0],[1,2,0]],[[5,5,0],[7,5,0],[7,7,0],[5,7,0]]],False)
        self.assertEqual(len(g.deleted),2);self.assertEqual(len(g.Surfaces),2)
        self.assertEqual(g.Surfaces[0].Name,'Unrelated');self.assertEqual(g.Surfaces[1].Area,95)
        self.assertEqual(g.Surfaces[1].Name,'RCFSlab30Part1')
    def test_empty_remote_surface_slice_can_be_none(self):
        class RemoteList(list):
            def __getitem__(self,key):
                if isinstance(key,slice) and not self:return None
                return super().__getitem__(key)
        g=Fake();g.Surfaces=RemoteList();ns=self.runtime(g)
        self.assertEqual(ns['_surfaces'](),{})
        ns['_slab'](1,[[0,0,0],[10,0,0],[10,10,0],[0,10,0]],[],False)
        self.assertEqual(len(g.Surfaces),1)
        self.assertEqual(g.Surfaces[0].Name,'RCFSlab1Part1')

    def test_nonempty_remote_surface_slice_none_is_not_treated_as_empty(self):
        class BrokenList(list):
            def __getitem__(self,key):
                if isinstance(key,slice):return None
                return super().__getitem__(key)
        g=Fake();g.Surfaces=BrokenList(g.Surfaces);ns=self.runtime(g)
        with self.assertRaisesRegex(RuntimeError,'reports 1 surfaces'):
            ns['_surfaces']()
        self.assertEqual(g.deleted,[])

    def test_units_read_symbolic_enum_names_and_reject_wrong_or_unknown_units(self):
        g=Fake();ns=self.runtime(g)
        g.Project.UnitLength=types.SimpleNamespace(value=2,strvalue='m')
        g.Project.UnitForce=types.SimpleNamespace(value=7,strvalue='kN')
        ns['_check_units']()
        g.Project.UnitLength=types.SimpleNamespace(value=2,enum_dict={'mm':0,'m':2})
        ns['_check_units']()
        g.Project.UnitLength=types.SimpleNamespace(value=2,strvalue='mm')
        with self.assertRaisesRegex(RuntimeError,'must be m, got mm'):ns['_check_units']()
        g.Project.UnitLength=types.SimpleNamespace(value=2)
        with self.assertRaisesRegex(RuntimeError,'numeric unit codes are not guessed'):ns['_check_units']()
        self.assertEqual(g.deleted,[])

    def test_password_length_is_checked_in_utf8_bytes_without_exposing_password(self):
        import ast
        tree=ast.parse(RUNTIME.read_text(encoding='utf-8-sig'))
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_server_password')
        ns={'os':types.SimpleNamespace(environ={}),
            'getpass':types.SimpleNamespace(getpass=lambda prompt:'abc')}
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(RUNTIME),'exec'),ns)
        for password in ['a','abc','x'*57]:
            ns['os'].environ={'PLAXIS_PASSWORD':password}
            with self.assertRaisesRegex(RuntimeError,'4..56 UTF-8 bytes'):
                ns['_server_password']()
        for password in ['abcd','x'*56,'я'*2]:
            ns['os'].environ={'PLAXIS_PASSWORD':password}
            self.assertEqual(ns['_server_password'](),password)

    def test_missing_scripting_module_writes_error_log_without_connecting(self):
        import subprocess, tempfile
        with tempfile.TemporaryDirectory() as folder:
            script=Path(folder)/'startup.py'
            script.write_text(RUNTIME.read_text(encoding='utf-8-sig'),encoding='utf-8')
            # -S excludes third-party packages; failure must precede all server calls.
            result=subprocess.run([sys.executable,'-I','-S',str(script)],
                input='',capture_output=True,text=True,timeout=10)
            self.assertNotEqual(result.returncode,0)
            log=script.with_suffix('.error.log')
            self.assertTrue(log.exists())
            report=log.read_text(encoding='utf-8')
            self.assertIn('PLAXIS scripting module is unavailable',report)
            self.assertIn('Python: ',report)
            self.assertNotIn('Connecting to PLAXIS Input',result.stdout)

    def test_unique_hole_area_works_without_boundary_with_conservation(self):
        g=Fake();base=g.intersect
        def no_vertices(*args):
            result=base(*args);del result[1].Points;return result
        g.intersect=no_vertices;ns=self.runtime(g)
        ns['_slab'](30,[[0,0,0],[10,0,0],[10,10,0],[0,10,0]],
            [[[1,1,0],[2,1,0],[2,2,0],[1,2,0]]],False)
        self.assertEqual(len(g.deleted),1)
        self.assertEqual(g.Surfaces[0].Name,'Unrelated')
        self.assertEqual(g.Surfaces[1].Area,99)

    def test_unknown_geometry_with_inconsistent_area_stops_before_any_delete(self):
        g=Fake();base=g.intersect
        def bad(*args):
            result=base(*args);del result[1].Points;result[1].Area=3;return result
        g.intersect=bad;ns=self.runtime(g)
        with self.assertRaisesRegex(RuntimeError,'before deletion') as error:
            ns['_slab'](30,[[0,0,0],[10,0,0],[10,10,0],[0,10,0]],[[[1,1,0],[2,1,0],[2,2,0],[1,2,0]]],False)
        self.assertEqual(g.deleted,[])
        self.assertIn('"hole_area": 1.0',str(error.exception))
        self.assertIn('"vertex_count": null',str(error.exception))

    def test_equal_area_parts_without_boundary_are_not_guessed(self):
        g=Fake();base=g.intersect
        def ambiguous(*args):
            result=base(*args);del result[1].Points;return result
        g.intersect=ambiguous;ns=self.runtime(g)
        with self.assertRaisesRegex(RuntimeError,'before deletion'):
            ns['_slab'](30,[[0,0,0],[10,0,0],[10,10,0],[0,10,0]],
                [[[0,0,0],[5,0,0],[5,10,0],[0,10,0]]],False)
        self.assertEqual(g.deleted,[])

if __name__=='__main__':unittest.main()
