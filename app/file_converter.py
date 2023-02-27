'''
CAD Query library
repository - https://github.com/CadQuery/cadquery
documentation - https://cadquery.readthedocs.io/en/latest/intro.html 
examples - https://cadquery.readthedocs.io/en/latest/examples.html
cheatsheet - https://cadquery.readthedocs.io/en/latest/_static/cadquery_cheatsheet.html
'''

import cadquery as cq

class FileConverter:
    
    def export_step_to_stl(step_file_path, stl_file_path, tolerance=0.001, angularTolerance=0.001, ascii=False):
        model = cq.importers.importStep(step_file_path)
        result = model.box(1.0,2.0,3.0).val()
        result.exportStl(fileName=stl_file_path, tolerance=tolerance, angularTolerance=angularTolerance, ascii=ascii)
