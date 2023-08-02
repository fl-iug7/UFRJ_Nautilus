import pandas as pd

#Classe que implementa um modelo de AUV
class Auv:
    
    #Método construtor da classe
    def __init__(self, num_thursters: int, sensores: list, ano: int, nome: str, time: list):
        self.num_thursters = num_thursters
        self.sensores = sensores
        self.ano = ano
        self.nome = nome
        self.time = time
    
    
    #Método que será utilizado na função "exibir_ranking", ele retornará o parâmetro utilizado para ordenar a lista de AUVs
    def pegar_ano(self):
        return self.ano


#Classe que contém os métodos
class Metodo():
    
    #Método que exibe os AUVs em uma tabela
    def exibir_auvs(lista_auv):
        #Inicialização do dicionário que será utilizado como base para a construção do dataset (tabela)
        dados = {
            "Número de thursters": [],
            "Sensores": [],
            "Ano": [],
            "Nome": [],
            "Time": []
        }
        
        #Adicionando os elementos ao dicionário dados
        for elemento in lista_auv:
            dados["Número de thursters"].append(elemento.num_thursters)
            dados["Sensores"].append(elemento.sensores)
            dados["Ano"].append(elemento.ano)
            dados["Nome"].append(elemento.nome)
            dados["Time"].append(elemento.time)
        
        tabela = pd.DataFrame(dados)
        print(tabela.head())
        
        return print()
    
    
    #Método que exibe as informações de um AUV específico
    def exibir_auv(auv):
        
        print("-Número de thursters:", auv.num_thursters)
        
        print("-Sensores:", end="")
        for elemento in auv.sensores:
            print(f" {elemento}", end=" ")
            
        print("\n-Ano:", auv.ano)
        
        print("-Nome:", auv.nome)
        
        print("-Time:", end="")
        for elemento in auv.time:
            print(f" -{elemento}", end=" ")
            
        return print("\n")
    
    
    #Método que ordena os AUVs, do mais novo para o mais antigo
    def exibir_ranking(self, lista_auv):
        #Ordendação da lista por meio da função "sorted" e do método "pegar_ano" da classe "Auv"
        lista_ordenada = sorted(lista_auv, key=Auv.pegar_ano, reverse=True)
        
        for index in range(len(lista_ordenada)):
            print(f"{index + 1}. {lista_ordenada[index].nome}")
        
        return print()


    #Método que exibe o time que participou da competição
    def exibir_time(auv):
        for index in range(len(auv.time)):
            print(f"-Membro {index + 1}: {auv.time[index]}")
        
        return print()


#Inicializando os AUVs
auv1 = Auv(num_thursters=6, sensores=["Sensor de Pressão","IMU"], ano=2020, nome="BrHUE 2020", time=["Gustavo R. Villela", "Ana Clara L. Cruz", "Felipe B. Costa", "Ramon Christian Mendes", "Lara F. de Amorim", "Vitor A. Pavani", "Claudio M. de Farias"])
auv2 = Auv(num_thursters=8, sensores=["Sensor PCB","Sensor de Velocidade"], ano=2022, nome="Lua", time=["Victor P. Siqueira", "Lidia G. Paúra", "Augusto L. Rodrigues", "Felipe O. G. da Silva", "Matheus R. de Souza", "Claudio M. de Farias"])

#Inicializando uma lista que contém os AUVs
lista_auv = [auv1, auv2]

#Print dos requerimentos exigidos no trabalho
print("Exibição dos Auvs em uma tabela:")
Metodo.exibir_auvs(lista_auv)

print(f"Exibição do AUV {lista_auv[0].nome}:")
Metodo.exibir_auv(lista_auv[0])

print("Exibição do ranking dos AUVs:")
Metodo.exibir_ranking(Auv, lista_auv)

print(f"Exibição do time do AUV {lista_auv[0].nome}")
Metodo.exibir_time(auv1)