import copy
import threading
import unittest
import tempfile
import json
from pathlib import Path
from unittest.mock import patch
from unittest.mock import Mock
from field_console import analyze_intent, intent
from rover_web.evacuation import default_map, validate_map, plan_route, Evacuation

class FieldTests(unittest.TestCase):
    def test_saved_state_survives_restart_and_chat(self):
        with tempfile.TemporaryDirectory() as folder, patch('rover_web.evacuation.Path.home', return_value=Path(folder)):
            e=Evacuation(Mock())
            e.configure({'map':default_map(),'pose':{'cell':[0,0],'heading':1}})
            e=Evacuation(Mock())
            self.assertTrue(e.pose_valid)
            before=copy.deepcopy(e.model)
            e.new_dialog()
            self.assertTrue(e.pose_valid)
            self.assertEqual(before,e.model)
            e.running=True;e.save()
            self.assertFalse(Evacuation(Mock()).pose_valid)

    def test_saved_map_is_reusable_for_unlimited_new_missions(self):
        with tempfile.TemporaryDirectory() as folder, patch('rover_web.evacuation.Path.home', return_value=Path(folder)):
            model=default_map()
            model['markers']={'tree':[0,1]}
            model['person']={'cell':[0,1],'facing':1}
            e=Evacuation(Mock())
            e.configure({'map':model})
            committed=copy.deepcopy(e.model)
            for _ in range(3):
                e.pose={'cell':[4,2],'heading':3}
                e.pose_valid=False
                plan=e.prepare('person')
                self.assertEqual(plan['start'],{'cell':[0,0],'heading':1})
                self.assertEqual(e.model,committed)
            reloaded=Evacuation(Mock())
            self.assertEqual(reloaded.model,committed)
            reloaded.new_dialog()
            self.assertEqual(reloaded.model,committed)
            self.assertEqual(reloaded.pose,{'cell':[0,0],'heading':1})
            self.assertTrue(reloaded.pose_valid)

    def test_saved_map_cannot_occupy_permanent_start(self):
        e=Evacuation.__new__(Evacuation)
        e.lock=threading.RLock();e.running=False;e.model=default_map();e.initial_model=default_map()
        e.pose={'cell':[1,0],'heading':0};e.pose_valid=True;e.discovered=set();e.plan=None;e.save=Mock()
        model=default_map();model['markers']={'tree':[0,0]}
        with self.assertRaisesRegex(ValueError,'Стартовая клетка'):
            e.configure({'map':model})

    def test_return_faces_fixed_direction(self):
        plan=plan_route(default_map(),{'cell':[0,1],'heading':3},'collection')
        self.assertEqual(plan['steps'][-1]['pose'],{'cell':[0,0],'heading':1})

    def test_keywords(self):
        for word, key in [('аварию','accident'),('бензовоз','tanker'),('дерево','tree'),('остановку','bus_stop'),('обломки','rubble')]:
            self.assertEqual(intent(f'Найди {word} у синего дома'),(key,'blue_area'))
        phrases = {
            'Пострадавший рядом с голубым домом, путь перекрывает ДТП': ('accident', 'blue_area'),
            'У голубого здания находится топливная цистерна': ('tanker', 'blue_area'),
            'Возле лазурного строения упало дерево': ('tree', 'blue_area'),
            'В окрестностях желтого здания обнаружен завал': ('rubble', 'yellow_area'),
            'У золотистого дома находится остановочный павильон': ('bus_stop', 'yellow_area'),
            'За мостами произошла авария': ('accident', 'bridge'),
            'У переправы стоит бензовоз': ('tanker', 'bridge'),
            'Берёза упала возле голубого корпуса': ('tree', 'blue_area'),
            'У синей постройки лежит сосна': ('tree', 'blue_area'),
            'Рядом с лазурным сооружением повалило дуб': ('tree', 'blue_area'),
            'Клён находится возле жёлтого здания': ('tree', 'yellow_area'),
            'У лимонной постройки растёт тополь': ('tree', 'yellow_area'),
            'Автоцистерна стоит около янтарного корпуса': ('tanker', 'yellow_area'),
            'После столкновения у небесного дома перекрыта дорога': ('accident', 'blue_area'),
            'Возле эстакады обнаружены руины': ('rubble', 'bridge'),
            'У путепровода находится остановочный пункт': ('bus_stop', 'bridge'),
        }
        for text, expected in phrases.items():
            self.assertEqual(intent(text), expected)
        for text in ('авария', 'авария дерево синий дом', 'авария синий дом мост'):
            with self.assertRaises(ValueError): intent(text)

    def test_missing_keywords_fall_back_without_error(self):
        self.assertEqual(analyze_intent('поехали выполнять задание'), ([], [], []))
        objects, places, keywords = analyze_intent('Найди берёзу')
        self.assertEqual(objects, ['tree'])
        self.assertEqual(places, [])
        self.assertEqual(keywords, ['дерево'])

    def test_synonyms_are_visible_as_keywords(self):
        self.assertEqual(analyze_intent('ДТП у голубого дома')[2],
                         ['авария', 'синий дом', 'голубой дом'])
        self.assertEqual(analyze_intent('Цистерна возле янтарного здания')[2],
                         ['бензовоз', 'жёлтый дом', 'янтарный дом'])
        self.assertEqual(analyze_intent('Завал у переправы')[2],
                         ['обломки', 'мост', 'переправа'])

    def test_fixed_and_movable(self):
        m=default_map()
        self.assertEqual(m['cells'][6]['kind'],'road')
        m['markers']={'rubble':[0,3],'bus_stop':[2,3]}
        self.assertEqual(validate_map(m)['markers'],m['markers'])
        m['markers']['tree']=[1,3]
        with self.assertRaises(ValueError): validate_map(m)

    def test_optional_objects_and_deletion(self):
        m=default_map()
        self.assertEqual(validate_map(m)['markers'],{})
        for key in ('accident','tanker','tree','bus_stop','rubble'):
            m['markers']={key:[0,3]}
            self.assertEqual(validate_map(m)['markers'],{key:[0,3]})
        m['markers']={}
        self.assertEqual(validate_map(m)['markers'],{})

    def test_target_qr_can_be_placed_in_fixed_cell(self):
        for target, facing, goal in [([3,4],2,[2,4]),([4,3],3,[4,2]),([4,4],3,[4,2])]:
            m=default_map()
            m['markers']={'accident':target}
            m['person']={'cell':target,'facing':facing}
            saved=validate_map(m)
            self.assertEqual(saved['markers']['accident'],target)
            self.assertEqual(saved['person'],{'cell':target,'facing':facing})
            plan=plan_route(saved,{'cell':[0,0],'heading':1},'person')
            self.assertEqual(plan['goal'],goal)

    def test_routes_and_qr_facing(self):
        m=default_map();m['markers']={'rubble':[2,2]}
        for facing in range(4):
            m['person']={'cell':[0,3],'facing':facing}
            try: p=plan_route(m,{'cell':[0,0],'heading':1},'person')
            except ValueError: continue
            self.assertEqual(p['steps'][-1]['pose']['heading'],(facing+2)%4)
            for a,b in zip(p['path'],p['path'][1:]):self.assertEqual(abs(a[0]-b[0])+abs(a[1]-b[1]),1)

    def test_camera_and_map_must_match_before_chat(self):
        e=Evacuation.__new__(Evacuation);e.lock=threading.RLock();e.model=default_map()
        e.model['markers']={'tree':[0,1],'accident':[1,0]};e.pose={'cell':[0,0],'heading':1}
        e.discovered=set();e.vision_confirmed=set();e.plan=None;e.running=True;e.event=Mock()
        self.assertEqual(e.expected_camera_object(),'tree')
        self.assertFalse(e.confirm_camera_object('accident','ДТП',.9,'image','wrong'))
        self.assertEqual(e.event.call_count,0)
        self.assertTrue(e.confirm_camera_object('tree','Дерево',.91,'image','right'))
        self.assertEqual(e.discovered,{'tree'})
        self.assertEqual(e.event.call_count,2)
        self.assertEqual(e.event.call_args_list[0].args,('',))
        self.assertEqual(e.event.call_args_list[0].kwargs['imageData'],'image')
        self.assertIn('Дерево', e.event.call_args_list[1].args[0])
        self.assertFalse(e.confirm_camera_object('tree','Дерево',.99,'image2','repeat'))
        self.assertEqual(e.event.call_count,2)

    def test_front_object_is_reported_after_camera_timeout(self):
        e=Evacuation.__new__(Evacuation);e.lock=threading.RLock();e.model=default_map()
        e.model['markers']={'accident':[0,1]};e.model['person']=None
        e.pose={'cell':[0,0],'heading':1};e.discovered=set();e.plan=None;e.event=Mock()
        self.assertEqual(e.discover(),{'accident'})
        self.assertEqual(e.discovered,{'accident'})
        self.assertIn('аварию перед собой',e.event.call_args.args[0])
        self.assertEqual(e.event.call_args.kwargs['map']['markers'],{'accident':[0,1]})
        self.assertEqual(e.discover(),set())

if __name__=='__main__':unittest.main()
