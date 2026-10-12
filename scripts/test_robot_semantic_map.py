import copy
import io
import tempfile
import unittest
import uuid
from pathlib import Path
from project_store import ProjectStore, CATEGORIES
from semantic_dataset import export_dataset, read_dataset
from robot_semantic_map import adapt, convert
from test_project_store import SCENE, TMP, payload


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(dir=TMP);self.root=Path(self.temp.name)
        self.store=ProjectStore(self.root/'store',SCENE);self.p=payload(count=2)
        self.zone='zone-'+str(uuid.uuid4())
        self.p['spatial_context']=dict(schema='scenesem.spatial-context.v2',zones=[dict(zone_id=self.zone,name='Synthetic waiting area',kind='functional_area',parent_id=None,source='manual',verification='unverified',notes='',boundary=dict(type='aabb',min=[0,0,0],max=[5,4,3]))],memberships=[dict(object_id=self.p['objects'][0]['object_id'],zone_id=self.zone,source='manual',verification='unverified',notes='')])
        self.store.save(self.p);self.zip=self.root/'source.zip'
        with self.zip.open('xb') as f:export_dataset(self.store,1,f,CATEGORIES)
        self.data=read_dataset(self.zip)

    def tearDown(self):self.temp.cleanup()

    def test_mapping_unknowns_and_projection(self):
        before=copy.deepcopy(self.data);r=adapt(self.data,up_axis='z')
        self.assertEqual(self.data,before)
        self.assertEqual(r['objects'][0]['id'],self.p['objects'][0]['object_id'])
        self.assertEqual(r['objects'][0]['region_id'],self.zone)
        self.assertIsNone(r['objects'][1]['region_id'])
        self.assertEqual(r['objects'][0]['bbox3d']['center'],[.5,.5,.5])
        self.assertEqual(r['regions'][0]['polygon2d'],[[0,0],[5,0],[5,4],[0,4]])
        self.assertEqual(r['regions'][0]['objects'],[r['objects'][0]['id']])
        self.assertEqual(r['places'],[])
        for field in ('centroid','last_seen','observation_count','embedding_ref','label_confidence'):
            self.assertIsNone(r['objects'][0][field])
        self.assertIsNone(adapt(self.data)['regions'][0]['polygon2d'])

    def test_conflicting_categories_and_orphans_are_not_guessed(self):
        d=copy.deepcopy(self.data);d['objects'][0]['region_ids'].append(d['regions'][1]['id']);d['objects'].pop()
        d['regions'][1]['category']='table'
        self.assertIsNone(adapt(d)['objects'][0]['label'])
        d=copy.deepcopy(self.data);d['objects'].pop()
        self.assertEqual(adapt(d)['limitations']['unassigned_source_region_ids'],[d['regions'][1]['id']])

    def test_bad_links_and_bounds_rejected(self):
        for mutate in (lambda d:d['objects'][0]['region_ids'].append('missing'),lambda d:d['regions'][0]['min'].__setitem__(0,float('nan')),lambda d:d['objects'][1]['region_ids'].__setitem__(0,d['regions'][0]['id'])):
            d=copy.deepcopy(self.data);mutate(d)
            with self.assertRaises(ValueError):adapt(d)

    def test_cli_core_source_unchanged_no_overwrite(self):
        before=self.zip.read_bytes();out=self.root/'robot.json'
        r=convert(self.zip,out,up_axis='z')
        self.assertEqual(len(r['source_dataset_sha256']),64)
        self.assertEqual(before,self.zip.read_bytes())
        with self.assertRaises(FileExistsError):convert(self.zip,out)


if __name__=='__main__':unittest.main()
