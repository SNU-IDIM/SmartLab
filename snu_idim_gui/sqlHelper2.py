# Python 3
# -*- coding: utf-8 -*-

import pymysql
import os
import csv
from datetime import datetime

class SqlHelper():
    def __init__(self, host="localhost", username="root", password="", port=3306, database=None, charset="utf8mb4"):
        self.host = host
        self.username = username
        self.password = password
        self.database = database
        self.port = port
        self.con = None
        self.cur = None

        try:
            self.con = pymysql.connect(host=self.host, user=self.username, passwd=self.password, port=self.port, charset=charset)
            self.cur = self.con.cursor()
            if database is not None:
                self.select_db(database)
        except:
            raise "[ERROR] DataBase connection error !!!"


    def select_db(self, database):
        self.cur.execute('CREATE DATABASE IF NOT EXISTS {}'.format(database))
        self.cur.execute('USE {}'.format(database))
        self.database = database
        print("[DEBUG] Selected DB: {}".format(self.database))


    def close(self):
        if not self.con:
            self.con.close()


    def get_version(self):
        self.cur.execute("SELECT VERSION()")
        version = self.get_one_data()[0]
        print("[DEBUG] MySQL Version: {}".format(version))
        return version


    def get_one_data(self):
        data = self.cur.fetchone()
        return data


    def fetch_one_data(self, tablename):
        sql = "SELECT * FROM {}".format(tablename)
        print('[SQL] {}'.format(sql))
        self.execute_commit(sql)
        return self.cur.fetchone()

    def create_table(self, tablename, attrdict, constraint):
        """
            example:
                attrdict   ：{'book_name':'varchar(200) NOT NULL'...}
                constraint ：PRIMARY KEY(`id`)
        """
        if self.is_exist_table(tablename):
            return
        sql = ''
        sql_mid = ''
        for attr,value in attrdict.items():
            # sql_mid += '`'+attr + '`'+' '+ value+','
            sql_mid += "{} {}, ".format(attr, value)#  '`'+attr + '`'+' '+ value+','
        sql += 'CREATE TABLE IF NOT EXISTS {} ('.format(tablename)
        sql += sql_mid + constraint 
        sql += ') ENGINE=InnoDB DEFAULT CHARSET=utf8mb4'
        print('[SQL] {}'.format(sql))
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
            error = 'MySQL execute failed! ERROR (%s): %s' %(e.args[0],e.args[1])
            print(error)

    def execute_sql_file(self, sql):
        if not os.path.isfile(sql):
            raise 'invalid input file path: %s' % sql
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
            error = 'MySQL execute failed! ERROR (%s): %s' %(e.args[0],e.args[1])
            raise error

    def execute_commit(self,sql=''):
        try:
            self.cur.execute(sql)
            self.con.commit()
        except pymysql.Error as e:
            self.con.rollback()
            error = 'MySQL execute failed! ERROR (%s): %s' %(e.args[0],e.args[1])
            print("error:", error)
            return error

    def insert(self, tablename, params):
        """
            example:
                mydb.select(table, param={"column1": data1})
                mydb.select(table, param={"column1": data1, "column2": data2})
        """
        if params is not None:
            if not isinstance(params, (dict, list, tuple)) or len(params) == 0:
                e = SQLValueError('expression insert', 'invalid values')
                raise e

            if isinstance(params, list):
                print("@@@@@@@@@@@@@@@@@@@")

            attrs_sql = ''
            if isinstance(params, dict):
                columns = [k.strip('\'"') for k in params.keys()]
                print(columns)
                attrs_sql = f"{','.join(columns)}"
                values = [v.decode('utf8') if isinstance(v, bytes) else v for v in params.values()]
                values_sql = f'{tuple(values)}'
                if values_sql[-2:].find(',') != -1:
                    values_sql = f'{tuple(values)}'[:-2] + ')'
            elif isinstance(params, list):
                list_keys = []
                list_values = []
                for data in params:
                    key = list(data.keys())[0]
                    value = list(data.values())[0]
                    
                    list_keys.append(key)
                    list_values.append(value)
                print(list_keys, list_values)
                attrs_sql = f"{','.join(list_keys)}"
                values_sql = f"{','.join(list_values)}"
                    



                    # attrs_sql = f"{','.join([columns])}"
                    # values = f"{','.join(goood)}"
                    # print(attrs_sql)
                    # print(list(data.keys())[0])
                    # print(list(data.values())[0])




        sql = 'INSERT INTO {} ({}) VALUES ({})'.format(tablename, attrs_sql, values_sql)
        print('[SQL] {}'.format(sql))
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
        print('[SQL] {}'.format(sql))
        records = self.execute_sql(sql)
        if records is None or len(records) == 0:
            return None
        if len(records[0]) == 1:
            return tuple([r for r, in records])

        columns = self.get_table_columns(tablename) if fields == '*' else fields
        data = tuple([dict(zip(columns, row)) for row in records])
        print('[DEBUG] Selected data: \n{}'.format(data))

        return data


    def delete(self, tablename, cond_dict):
        """
            example：
                params = {"name" : "caixinglong", "age" : "38"}
                mydb.delete(table, params)
        """
        consql = ' '
        if cond_dict!='':
            for k, v in cond_dict.items():
                if isinstance(v, str):
                    v = "\'" + v + "\'"
                consql = consql + tablename + "." + k + '=' + v + ' and '
        consql = consql + ' 1=1 '
        sql = "DELETE FROM %s where%s" % (tablename, consql)
        #print (sql)
        return self.execute_commit(sql)

    def update(self, tablename, attrs_dict, cond_dict):
        """
            example：
                params = {"name" : "caixinglong", "age" : "38"}
                cond_dict = {"name" : "liuqiao", "age" : "18"}
                mydb.update(table, params, cond_dict)
        """
        attrs_list = []
        consql = ' '
        for tmpkey, tmpvalue in attrs_dict.items():
            attrs_list.append("`" + tmpkey + "`" + "=" +"\'" + str(tmpvalue) + "\'")
        attrs_sql = ",".join(attrs_list)
        #print("attrs_sql:", attrs_sql)
        if cond_dict!='':
            for k, v in cond_dict.items():
                v = "\'" + str(v) + "\'"
                consql = consql + "`" + tablename +"`." + "`" + str(k) + "`" + '=' + v + ' and '
        consql = consql + ' 1=1 '
        sql = "UPDATE %s SET %s where%s" % (tablename, attrs_sql, consql)
        #print(sql)
        return self.execute_commit(sql)


    def drop_table(self, tablename):
        sql = "DROP TABLE {}".format(tablename)
        self.execute_commit(sql)


    def delete_table(self, tablename):
        sql = "DELETE FROM {}".format(tablename)
        self.execute_commit(sql)


    def is_exist_schema(self, database):
        result, = self.select("information_schema.SCHEMATA", "count(*)",["schema_name = '%s'" % database])
        return result > 0


    def is_exist_table(self, tablename):
        result, = self.select("information_schema.tables","count(*)",["table_name = '%s'" % tablename, "table_schema = '%s'" % self.database])
        return result > 0


    def is_exist_table_column(self, tablename, column_name):
        result, = self.select("information_schema.columns","count(*)",["table_name = '%s'" % tablename, "column_name = '%s'" % column_name, "table_schema = '%s'" % self.database])
        return result > 0


    def is_exist_table_rows(self, tablename, clauses):
        result = self.select(tablename, '*', conds=clauses)
        return False if result is None or len(result) == 0 else len(result[0]) > 0


    def get_table_columns(self, tablename):
        return self.select("information_schema.columns", "column_name", ["table_schema = '%s'" % self.database, "table_name = '%s'" % tablename])

    def get_table_columns_prim(self, tablename):
        return self.select("information_schema.columns", "column_name", ["table_schema = '%s'" % self.database, "table_name = '%s'" % tablename, "column_key = 'PRI'"])

    def add_column(self, tablename, col, tp):
        sql = "ALTER TABLE {} ADD {} {}".format(tablename, col, tp)
        self.execute_commit(sql)
        print("[DEBUG] New column is added !!! ({}: {})".format(col, tp))

    def delete_column(self, tablename, col):
        sql = "alter table %s drop column %s" % (tablename, col)
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
            print('table: %s not exists, by pass %s' % (tablename, file))
            return
        if not os.path.isfile(file):
            print('file: %s not exists, can not import to table %s' % (file, tbl))
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
    mysql = SqlHelper(host='localhost', username='root', password='0000', port=3306)
    mysql.get_version()

    mysql.select_db('SmartLab')

    if not mysql.is_exist_table('device_info'):
        mysql.create_table(tablename='device_info', attrdict={'id': 'int(11) AUTO_INCREMENT', 'time_stamp': 'timestamp'}, constraint="PRIMARY KEY(id)")
    

    device_list = ['R_001/cobot', 'R_001/amr', 'instron', 'MS', 'time_stamp']

    for device_id in device_list:
        if device_id.find('/') != -1:   device_id = device_id.split('/')[1]
        try:
            list(mysql.get_table_columns(tablename='device_info')).index(device_id)
            print("[DEBUG] Column already exists !!! ({})".format(device_id))

        except:
            mysql.add_column('device_info', device_id, 'varchar(32)')
            print("[DEBUG] Column not exists !!! ({})".format(device_id))

    print(mysql.get_table_columns(tablename='device_info'))
    mysql.select('device_info')

    test = [{'time_stamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')}, {'cobot': "TEST"}]

    # mysql.insert('device_info', {'time_stamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')})
    mysql.insert('device_info', test)
    mysql.select('device_info')

    # if not mysql.is_exist_table('device_info'):
        
    #     mysql.create_table(tablename='device_info', attrdict={'id': 'int(11) AUTO_INCREMENT', 'time_stamp': 'timestamp', 'time_stamp2': 'timestamp'}, constraint="PRIMARY KEY(id)")
    #     mysql.create_table(tablename='device_info2', attrdict={'id': 'int(11) AUTO_INCREMENT', 'time_stamp': 'timestamp', 'time_stamp2': 'timestamp'}, constraint="PRIMARY KEY(id)")

    # print(mysql.get_table_columns(tablename='device_info'))
    # mysql.insert('device_info', {'time_stamp': '2021-04-25 00:14:10', 'time_stamp2': '2021-04-25 00:14:10'})
    # mysql.insert('device_info2', {'time_stamp': '2021-04-25 00:14:10', 'time_stamp2': '2021-04-25 00:14:10'})
    # for i in mysql.select('device_info'):
    #     print(type(i))
    # for i in mysql.select('device_info2'):
    #     print(type(i))