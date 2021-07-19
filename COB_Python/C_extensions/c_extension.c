//c_extension.c

#include "Python.h"
#include "stdlib.h"
#include "omp.h"

PyObject *dot_product(PyObject *self, PyObject *args) {

    PyObject *mat1, *mat2;
    
    if(!PyArg_ParseTuple(args, "O|O", &mat1, &mat2)) {
        return NULL;
    }
    
    //getting dimensions of our lists
    int mat1_rows, mat1_columns, mat2_rows, mat2_columns;
    mat1_rows = PyObject_Length(mat1);
    mat1_columns = PyObject_Length(PyList_GetItem(mat1, 0));
    mat2_rows = PyObject_Length(mat2);
    mat2_columns = PyObject_Length(PyList_GetItem(mat2, 0));
    
    
    PyObject *pyResult = PyList_New(mat1_rows);
    PyObject *item, *mat1_current_row;
    int total;
    for(int i = 0; i < mat1_rows; i++) {
        item = PyList_New(mat2_columns);
        mat1_current_row = PyList_GetItem(mat1, i);
        for(int j = 0; j < mat2_columns; j++) {
            total = 0;
            for (int k = 0; k < mat2_rows; k++) {
                total += (int)PyLong_AsLong(PyList_GetItem(mat1_current_row, k)) * \
                (int)PyLong_AsLong(PyList_GetItem(PyList_GetItem(mat2, k), j));
            }
            PyList_SetItem(item, j, PyLong_FromLong(total));
        }
        PyList_SetItem(pyResult, i, item);
    }
    
    return Py_BuildValue("O", pyResult);
    
    
    static PyMethodDef module_methods[] = {
        {"dot_product", (PyCFunction) dot_product, METH_VARARGS, "Calculates dot product of two matrices"}
    };

    static struct PyModuleDef cModPyDem = {
        PyModuleDef_HEAD_INIT,
        "c_extension",
        "",
        -1,
        module_methods
    };


    PyMODINIT_FUNC PyInit_c_extension(void) {
        return PyModule_Create(&cModPyDem);
    }
    
   
    
}