import copy
import io
import json
import tempfile
import unittest
import uuid
import zipfile
from pathlib import Path
from project_store import ProjectStore, CATEGORIES
from test_project_store import SCENE, TMP, payload
from bundle_import import BundleImporter
from semantic_dataset import validate_business, business_quality, check_business_at_date, export_dataset, read_dataset, encode, sha


def fixture(oid):
    return dict(schema='scenesem.business-context.v1', attributes=[dict(attribute_id='attr-'+str(uuid.uuid4()),
        subject_kind='object',subject_id=oid,name='服务类型',value='Synthetic only',basis='simulation',source='Test specification v1',
        recorder='',observed_on=None,valid_from='2026-09-01',valid_until='2026-09-30',verification='unverified')])


class BusinessTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=TMP); self.root=Path(self.tmp.name)
        self.store=ProjectStore(self.root/'source',SCENE); self.p=payload(count=2)
        self.b=fixture(self.p['objects'][0]['object_id']); self.p['business_context']=self.b
    def tearDown(self): self.tmp.cleanup()
    def test_contract(self):
        validate_business(self.b)
        for key,value in [('basis','confirmed'),('source',''),('valid_until','2026-08-31'),('valid_from','2026-02-30'),('observed_on','0000-01-01'),('subject_id','wrong'),('verification','verified')]:
            bad=copy.deepcopy(self.b);bad['attributes'][0][key]=value
            with self.assertRaises(ValueError):validate_business(bad)
        field=copy.deepcopy(self.b);field['attributes'][0]['basis']='field_report'
        with self.assertRaises(ValueError):validate_business(field)
        field['attributes'][0].update(recorder='Synthetic witness',observed_on='2026-09-14');validate_business(field)
        unknown=copy.deepcopy(self.b);unknown['attributes'][0].update(basis='unknown',value='');validate_business(unknown)
    def test_old_client_guard_and_restore(self):
        saved=self.store.save(self.p)['document']; old=copy.deepcopy(self.p);old['expected_revision']=1;del old['business_context']
        with self.assertRaises(ValueError):self.store.save(old)
        old['business_context']=None;self.store.save(old)
        self.assertNotIn('business_context',self.store.load()['document']);self.assertEqual(self.store.load(1)['document'],saved)
    def test_date_checks_preserve_project_and_dataset_quality(self):
        saved=self.store.save(self.p)['document']
        for day,expected in [('2026-08-31','not_started'),('2026-09-30','within_declared_period'),('2026-10-01','expired')]:
            report=check_business_at_date(self.b,self.p['objects'],None,day)
            self.assertEqual(report['rows'][0]['temporal'],expected);self.assertFalse(report['task_qualification'])
        for day in ('2026-02-30','0000-01-01','2026-9-14'):
            with self.assertRaises(ValueError):check_business_at_date(self.b,self.p['objects'],None,day)
        with self.assertRaises(ValueError):check_business_at_date(self.b,self.p['objects'],None,'2026-09-14',True)
        self.assertEqual(self.store.load()['document'],saved)
        out=io.BytesIO();export_dataset(self.store,1,out,CATEGORIES)
        self.assertEqual(read_dataset(io.BytesIO(out.getvalue()))['quality']['business']['current_applicability'],'not_evaluated')
    def test_quality_preserves_missing_and_multiple_claims(self):
        duplicate=copy.deepcopy(self.b['attributes'][0]);duplicate['attribute_id']='attr-'+str(uuid.uuid4());duplicate['value']='Different synthetic claim';self.b['attributes'].append(duplicate)
        q=business_quality(self.b,[],None);self.assertEqual(len(q['multiple_claim_attribute_ids']),2);self.assertEqual(len(q['missing_subject_attribute_ids']),2);self.assertFalse(q['source_content_verified'])
    def test_v3_export_roundtrip_and_forged_quality(self):
        saved=self.store.save(self.p)['document'];out=io.BytesIO();export_dataset(self.store,1,out,CATEGORIES);read=read_dataset(io.BytesIO(out.getvalue()))
        self.assertEqual(read['manifest']['schema'],'scenesem.semantic-dataset.v3');self.assertEqual(read['business'],self.b);self.assertEqual(read['objects'],saved['objects']);self.assertNotIn('spatial',read)
        with zipfile.ZipFile(io.BytesIO(out.getvalue())) as z:files={n:z.read(n) for n in z.namelist()}
        q=json.loads(files['quality.json']);q['business']['source_content_verified']=True;files['quality.json']=encode(q)
        m=json.loads(files['manifest.json']);m['files']['quality.json']=dict(bytes=len(files['quality.json']),sha256=sha(files['quality.json']));files['manifest.json']=encode(m)
        forged=io.BytesIO()
        with zipfile.ZipFile(forged,'w') as z:
            for name,raw in files.items():z.writestr(name,raw)
        with self.assertRaises(ValueError):read_dataset(io.BytesIO(forged.getvalue()))
    def test_bundle_and_checkpoint_with_business(self):
        self.store.save(self.p);buf=io.BytesIO();self.store.export(1,buf)
        target=ProjectStore(self.root/'target',SCENE);imp=BundleImporter(target,self.root/'stage');preview=imp.preview(io.BytesIO(buf.getvalue()),len(buf.getvalue()))
        checkpoint=copy.deepcopy(self.p);checkpoint['business_context']['attributes'][0]['value']='Checkpoint synthetic'
        result=imp.commit(dict(import_id=preview['import_id'],expected_revision=0,checkpoint=checkpoint))
        self.assertEqual(result['document']['business_context'],self.b);self.assertEqual(target.load(result['checkpoint_revision'])['document']['business_context'],checkpoint['business_context'])


if __name__=='__main__':unittest.main()
