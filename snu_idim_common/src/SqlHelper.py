# Python 3
# -*- coding: utf-8 -*-

import pymysql
import os
import csv
from datetime import datetime
import json
import ast

class SqlHelper():
    def __init__(self, host="localhost", username="root", password="", port=3306, database=None, charset="utf8mb4", debug=False):
        self.host = host
        self.username = username
        self.password = password
        self.database = database
        self.port = port
        self.con = None
        self.cur = None

        self.debug = debug

        try:
            self.con = pymysql.connect(host=self.host, user=self.username, passwd=self.password, port=self.port, charset=charset)
            self.con.autocommit(True)
            self.cur = self.con.cursor()
            if database is not None:
                self.select_db(database)
        except:
            raise "[ERROR] DataBase connection error !!!"
    
    def __del__(self):
        if not self.con:
            self.con.close()
    
    def printSQL(self, sql):
        if self.debug == True:
            print('[SQL] {}'.format(sql))


    def select_db(self, database):
        self.cur.execute('CREATE DATABASE IF NOT EXISTS {}'.format(database))
        self.cur.execute('USE {}'.format(database))
        self.database = database
        self.printSQL(self.database)


    def close(self):
        if not self.con:
            self.con.close()


    def get_version(self):
        self.cur.execute("SELECT VERSION()")
        version = self.get_one_data()[0]
        self.printSQL(version)
        return version


    def get_one_data(self):
        data = self.cur.fetchone()
        return data


    def fetch_one_data(self, tablename):
        sql = "SELECT * FROM {}".format(tablename)
        self.printSQL(sql)
        self.execute_commit(sql)
        return self.cur.fetchone()


    def create_table(self, tablename, attrdict, constraint):
        """
            example:
                attrdict   ：{'book_name':'varchar(200) NOT NULL'...}
                constraint ："PRIMARY KEY('id')"
        """
        if self.is_exist_table(tablename):
            return
        sql_mid = ''
        for attr,value in attrdict.items():
            sql_mid += "{} {}, ".format(attr, value)
        sql_mid += constraint
        sql = "CREATE TABLE IF NOT EXISTS {} ({}) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4".format(tablename, sql_mid)
        self.printSQL(sql)
        self.execute_commit(sql)


    def recreate_table(self, tablename, attrdict, constraint):
        if not self.is_exist_table(tablename):
            return self.create_table(tablename, attrdict, constraint)

        self.backup_table(tablename, tablename + '.csv')
        self.delete_table(tablename)
        self.create_table(tablename, attrdict, constraint)
        self.restore_table(tablename, tablename + '.csv')
        os.remove(tablename + '.csv')
    

    def execute_sql(self, sql=''):
        try:
            self.cur.execute(sql)
            records = self.cur.fetchall()
            return records
        except pymysql.Error as e:
            error = 'MySQL execute failed! ERROR ({}): {}'.format(e.args[0], e.args[1])
            print("[ERROR] {}".format(error))
            return error


    def execute_sql_file(self, sql):
        if not os.path.isfile(sql):
            raise 'invalid input file path: {}'.format(sql)
        stmts = []
        stmt = ''
        delimiter = ';'
        commentsblock = False
        with open(sql, 'r', encoding='utf-8') as f:
            data = f.readlines()
            for lineno, line in enumerate(data):
                if not line.strip():
                    continue
                if line.startswith('--'):
                    continue
                if line.startswith('/*'):
                    commentsblock = True
                    continue
                if line.endswith('*/'):
                    commentsblock = False
                    continue
                if 'DELIMITER' in line:
                    delimiter = line.split()[1]
                    continue
                if delimiter not in line:
                    stmt += line.replace(delimiter, ';')
                    continue

                if stmt:
                    stmt += line
                    stmts.append(stmt.strip())
                    stmt = ''
                else:
                    stmts.append(line.strip())
        try:
            for s in stmts:
                self.cur.execute(s)
            self.con.commit()
        except pymysql.Error as e:
            self.con.rollback()
            error = 'MySQL execute failed! ERROR ({}): {}' %(e.args[0], e.args[1])
            print("[ERROR] {}".format(error))
            raise error


    def execute_commit(self, sql=''):
        try:
            self.cur.execute(sql)
            self.con.commit()
        except pymysql.Error as e:
            error = 'MySQL execute failed! ERROR ({}): {}'.format(e.args[0], e.args[1])
            print("[ERROR] {}".format(error))
            self.con.rollback()
            return error


    def insert(self, tablename, params, conds=''):
        """
            example:
                mydb.select(table, param={"column1": data1})
                mydb.select(table, param={"column1": data1, "column2": data2})
        """
        if params is not None:
            if not isinstance(params, dict) or len(params) == 0:
                e = SQLValueError('expression insert', 'invalid values')
                raise e
            keys = []
            values = []
            for key in params:
                value = params[key]
                value = "\'{}\'".format(value) if isinstance(value, str) else value
                keys.append(key)
                values.append(value)

            attrs_sql = ','.join(keys)
            values_sql = ','.join(values)
            

        sql = 'INSERT INTO {}({}) VALUES ({}) {}'.format(tablename, attrs_sql, values_sql, conds)
        self.printSQL(sql)
        self.execute_commit(sql)


    def select(self, tablename, fields='*', conds='', order=''):
        """
            example：
                print mydb.select(table)
                print mydb.select(table, fields=["name"])
                print mydb.select(table, fields=["name", "age"])
                print mydb.select(table, fields=["age", "name"])
                print mydb.select(table, fields=["age", "name"], conds = ["name = 'usr_name'", "age < 30"])
        """
        sql = ','.join(fields) if isinstance(fields, list) else fields
        sql = 'SELECT {} FROM {} '.format(sql, tablename)

        consql = ''
        if conds != '':
            if isinstance(conds, list):
                conds = ' AND '.join(conds)
                consql = 'WHERE ' + conds
            else:
                consql = 'WHERE ' + conds

        sql += consql + order
        self.printSQL(sql)
        records = self.execute_sql(sql)
        if records is None or len(records) == 0:
            return None
        if len(records[0]) == 1:
            data = tuple([r for r, in records])
            # print('[DEBUG] Selected data: \n{}'.format(data))
            return data

        columns = self.get_table_columns(tablename) if fields == '*' else fields
        data = tuple([dict(zip(columns, row)) for row in records])
        # print('[DEBUG] Selected data: \n{}'.format(data))
        return data


    def delete(self, tablename, cond_dict):
        """
            example：
                params = {"name" : "caixinglong", "age" : "38"}
                mydb.delete(table, params)
        """
        consql = ''
        if cond_dict!='':
            for k, v in cond_dict.items():
                v = "'{}'".format(v)
                consql += "{} {}.{}={} AND ".format(consql, tablename, k, v) 
        consql = consql + '1=1'
        sql = "DELETE FROM {} WHERE {}".format(tablename, consql)
        self.printSQL(sql)
        return self.execute_commit(sql)


    def update(self, tablename, attrs_dict, cond_dict):
        """
            example：
                params = {"name" : "caixinglong", "age" : "38"}
                cond_dict = {"name" : "liuqiao", "age" : "18"}
                mydb.update(table, params, cond_dict)
        """
        attrs_list = []
        consql = ''
        for tmpkey, tmpvalue in attrs_dict.items():
            attrs_list.append("`" + tmpkey + "`" + "=" +"\'" + str(tmpvalue) + "\'")
        attrs_sql = ",".join(attrs_list)
        if cond_dict!='':
            for k, v in cond_dict.items():
                v = "\'" + str(v) + "\'"
                consql = consql + "`" + tablename +"`." + "`" + str(k) + "`" + '=' + v + ' and '
        consql = consql + ' 1=1 '
        sql = "UPDATE {} SET {} WHERE {}".format(tablename, attrs_sql, consql)
        self.printSQL(sql)
        return self.execute_commit(sql)


    def drop_table(self, tablename):
        sql = "DROP TABLE {}".format(tablename)
        self.printSQL(sql)
        self.execute_commit(sql)


    def delete_table(self, tablename):
        sql = "DELETE FROM {}".format(tablename)
        self.printSQL(sql)
        self.execute_commit(sql)


    def is_exist_schema(self, database):
        result, = self.select("information_schema.SCHEMATA", "count(*)", ["schema_name = '{}'".format(database)])
        return result > 0


    def is_exist_table(self, tablename):
        result, = self.select("information_schema.tables","count(*)", ["table_name = '{}'".format(tablename), "table_schema = '{}'".format(self.database)])
        return result > 0


    def is_exist_table_column(self, tablename, column_name):
        result, = self.select("information_schema.columns","count(*)", ["table_name = '{}'".format(tablename), "column_name = '{}'".format(column_name), "table_schema = '{}'".format(self.database)])
        return result > 0


    def is_exist_table_rows(self, tablename, clauses):
        result = self.select(tablename, '*', conds=clauses)
        return False if result is None or len(result) == 0 else len(result[0]) > 0


    def get_table_columns(self, tablename):
        return self.select("information_schema.columns", "column_name", ["table_schema = '{}'".format(self.database), "table_name = '{}'".format(tablename)])

    def get_table_columns_prim(self, tablename):
        return self.select("information_schema.columns", "column_name", ["table_schema = '{}'".format(self.database), "table_name = '{}'".format(tablename), "column_key = 'PRI'"])

    def add_column(self, tablename, col, tp):
        sql = "ALTER TABLE {} ADD {} {}".format(tablename, col, tp)
        self.printSQL(sql)
        self.execute_commit(sql)
        print("[DEBUG] New column is added !!! ({}: {})".format(col, tp))

    def delete_column(self, tablename, col):
        sql = "ALTER TABLE {} DROP COLUMN {}".format(tablename, col)
        self.printSQL(sql)
        self.execute_commit(sql)

    def backup_table(self, tablename, file):
        columns = self.get_table_columns(tablename)
        expdata = tuple([tuple([x for x in columns])])
        data = self.select(tablename, '*')
        expdata += tuple([tuple(d.values()) for d in data])
        with open(file, 'wt', encoding='utf-8') as bkf:
           csv_out = csv.writer(bkf)
           csv_out.writerows(expdata)

    def restore_table(self, tablename, file, existing_rows='update'):
        if not self.is_exist_table(tablename):
            print('Table: {} not exists, by pass {}'.format(tablename, file))
            return
        if not os.path.isfile(file):
            print('File: {} not exists, can not import to table {}'.format(file, tbl))
            return
        
        pri_keys = self.get_table_columns_prim(tablename)
        pri_keys = [k for k in pri_keys]
        col_names = []
        data_insert = []
        data_update = []
        with open(file, 'rt', encoding='utf-8') as bkf:
            csv_reader = csv.reader(bkf)
            pri_idx = []
            line_count = 0
            for row in csv_reader:
                if len(row) == 0:
                    continue
                if line_count == 0:
                    col_names = row
                    pri_idx = [ [k, col_names.index(k)] for k in pri_keys]
                    line_count += 1
                else:
                    clause = ''
                    pri_vals = []
                    for [k, i] in pri_idx:
                        clause += str(k) + '= \'' + str(row[i]) + '\' and '
                        pri_vals.append(row[i])
                    clause += ' 1=1 '
                    if self.is_exist_table_rows(tablename, clauses = clause):
                        if existing_rows == 'update':
                            data_update.append(row)
                    else:
                        data_insert.append(row)
        
        self.execute_commit('SET FOREIGN_KEY_CHECKS=0')
        if len(data_insert) > 0:
            datalist = [{c:v for c,v in zip(col_names, row)} for row in data_insert]
            self.insert_many(tablename, datalist=datalist)

        if len(data_update) > 0:
            datalist = [{c:v for c,v in zip(col_names, row)} for row in data_update]
            self.update_many(tablename, conkeys = pri_keys, datalist = datalist)

        self.execute_commit('SET FOREIGN_KEY_CHECKS=1')



if __name__ == "__main__":
    ## Connect to the database server
    mysql = SqlHelper(host='localhost', username='root', password='0000', port=3306, debug=True)

    ## Use database (Database: 'SmartLab')
    mysql.select_db('SmartLab')

    ## Create table if not exists (Table: 'device_info')
    device_list = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer1', 'printer2', 'printer3', 'printer4']
    # for device_id in device_list:
    #     if device_id.find('/') != -1:   device_id = device_id.split('/')[1]
    #     if not mysql.is_exist_table(device_id):
    #         mysql.create_table(tablename=device_id, attrdict={'id': 'int(11) AUTO_INCREMENT', 'time_stamp': 'timestamp', 'status': 'VARCHAR(256)'}, constraint="PRIMARY KEY(id)")
            
    if not mysql.is_exist_table('device_info'):
        mysql.create_table(tablename='device_info', attrdict={'id': 'int(11) AUTO_INCREMENT', 'time_stamp': 'timestamp'}, constraint="PRIMARY KEY(id)")

    
    # print(mysql.get_table_columns(tablename='cobot'))



    # # ## Create columns if not exists (Column: DEVICE_ID)
    device_list = ['R_001/cobot', 'R_001/amr', 'instron', 'MS', 'printer1', 'printer2', 'printer3', 'printer4']
    for device_id in device_list:
        if device_id.find('/') != -1:   device_id = device_id.split('/')[1]
        try:
            list(mysql.get_table_columns(tablename='device_info')).index(device_id)
            print("[DEBUG] Column already exists !!! ({})".format(device_id))
        except:
            mysql.add_column('device_info', device_id, 'varchar(1024)')
            print("[DEBUG] Column not exists !!! ({})".format(device_id))


    ## Insert data to table
    # column_list = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer1', 'printer2', 'printer3', 'printer4']
    # device_status = dict()
    # for device_id in column_list:
    #     if device_id.find('/') != -1:   device_id = device_id.split('/')[1]
    #     if device_id != 'id' and device_id != 'time_stamp':
    #         device_status[device_id] = json.dumps({'device_name': device_id, 'status': 'Idle'})
    # print(device_status)
    # mysql.insert('device_info', device_status)

    # ## Check existing columns and data
    # column_list = mysql.get_table_columns(tablename='device_info')
    # data = mysql.select('device_info')
    # print("[DEBUG] Column list: \n{}\n\nData in table ({}): \n{}".format(column_list, 'device_info', data))


    # ## Insert data to column (overwrite)
    # data = mysql.select('device_info', conds="id=(SELECT MIN(id) FROM device_info)")
    # print("[DEBUG] First data from table ({}): \n{}".format('device_info', data))
    # mysql.insert('device_info', new_data2)

    # # mysql.insert('device_info', {'id': 1}, conds='ON DUPLICATE KEY UPDATE amr = "test2"')
    # data = mysql.select('device_info', conds="id=(SELECT MAX(id) FROM device_info)")
    # print("[DEBUG] First data from table ({}): \n{}".format('device_info', data))



    '''
    ## Connect to the database server
    mysql = SqlHelper(host='localhost', username='root', password='0000', port=3306)

    ## Use database (Database: 'SmartLab')
    mysql.select_db('SmartLab')

    ## Check existing columns and data
    column_list = mysql.get_table_columns(tablename='result')
    print("[DEBUG] Column list: \n{} in table ({}): \n".format(column_list, 'result'))

    ## Insert data to column (overwrite)
    # mysql.add_column('result', 'status', 'varchar(256)')
    mysql.update('result', {'Status': 'Done'}, {"subject_name" : "yun_5"})
    
    subject_list = list(mysql.select('result', fields='subject_name'))

    test_list = ['yun_1', 'yun_2', 'yun_3', 'yun_4', 'yun_5', 'yun_6']
    for test in test_list:
        mysql.delete('result', {'subject_name': test})
        mysql.insert('result', {'subject_name': test, 'Status': 'Waiting'}, conds='ON DUPLICATE KEY UPDATE Status = "-"')
    
    mysql.insert('result', {'subject_name': 'DRY_TEST_0'}, conds='ON DUPLICATE KEY UPDATE Status = "Fabrication"')

        # try:
        #     idx = subject_list.index(test)
        #     mysql.update('result', {'Status': 'Waiting'}, {"subject_name" : test})
        # except:
        #     mysql.insert('result', {'subject_name': test, 'Status': 'Waiting'}, conds='ON DUPLICATE KEY UPDATE Status = "-"')
        #     print("NO")


    

    '''