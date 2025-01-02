#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

vector<vector<double>> transposeMatrix(const vector<vector<double>> mat)
{
    int rows = mat.size();
    int cols = mat[0].size();
    vector<vector<double>> result(cols, vector<double>(rows, 0.0));

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            result[j][i] = mat[i][j];
        }
    }

    return result;
}

vector<vector<double>> multiplyMatrices(const vector<vector<double>> mat1, const vector<vector<double>> mat2)
{
    int rows = mat1.size();
    int cols = mat2[0].size();
    int common = mat2.size();
    vector<vector<double>> result(rows, vector<double>(cols, 0.0));

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            for (int k = 0; k < common; ++k)
            {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }

    return result;
}

vector<vector<double>> invertMatrix(const vector<vector<double>> mat)
{
    int n = mat.size();
    vector<vector<double>> result(mat);

    vector<vector<double>> identity(n, vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i)
    {
        identity[i][i] = 1.0;
    }

    for (int i = 0; i < n; ++i)
    {
        double pivot = result[i][i];
        if (fabs(pivot) < 1e-9)
        {
            return {};
        }

        for (int j = 0; j < n; ++j)
        {
            result[i][j] /= pivot;
            identity[i][j] /= pivot;
        }

        for (int k = 0; k < n; ++k)
        {
            if (k != i)
            {
                double factor = result[k][i];
                for (int j = 0; j < n; ++j)
                {
                    result[k][j] -= factor * result[i][j];
                    identity[k][j] -= factor * identity[i][j];
                }
            }
        }
    }
    return identity;
}

vector<double> polyRegression(const vector<double> x, const vector<double> y, int degree)
{
    int n = x.size();
    vector<vector<double>> X(n, vector<double>(degree + 1, 1.0));

    // 다항식 확장
    for (int i = 0; i < n; ++i)
    {
        for (int j = 1; j <= degree; ++j)
        {
            X[i][j] = pow(x[i], j);
        }
    }

    vector<vector<double>> Xt = transposeMatrix(X);

    vector<vector<double>> XtX = multiplyMatrices(Xt, X);

    vector<vector<double>> XtX_inv = invertMatrix(XtX);

    vector<vector<double>> XtY(degree + 1, vector<double>(1, 0.0));
    for (int i = 0; i <= degree; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            XtY[i][0] += Xt[i][j] * y[j];
        }
    }

    vector<vector<double>> betaMat = multiplyMatrices(XtX_inv, XtY);

    vector<double> beta(degree + 1);
    for (int i = 0; i <= degree; ++i)
    {
        beta[i] = betaMat[i][0];
    }

    return beta;
}

int main()
{
    vector<double> x = {1, 2, 3, 4, 5};
    vector<double> y = {1.2, 1.9, 3.2, 3.8, 5.1};

    vector<double> coefficients_2 = polyRegression(x, y, 2);
    vector<double> coefficients_3 = polyRegression(x, y, 3);
    vector<double> coefficients_4 = polyRegression(x, y, 4);

    cout << "2 Coefficients:" << endl;
    for (int i = 0; i < coefficients_2.size(); ++i)
    {
        cout << "Beta[" << i << "] = " << coefficients_2[i] << endl;
    }

    cout << "3 Coefficients:" << endl;
    for (int i = 0; i < coefficients_3.size(); ++i)
    {
        cout << "Beta[" << i << "] = " << coefficients_3[i] << endl;
    }

    cout << "4 Coefficients:" << endl;
    for (int i = 0; i < coefficients_4.size(); ++i)
    {
        cout << "Beta[" << i << "] = " << coefficients_4[i] << endl;
    }
    return 0;
}
