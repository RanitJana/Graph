#include <bits/stdc++.h>
using namespace std;

namespace functions
{
    void bfs(int start, int V, vector<int> adj[], int *vis)
    {
        queue<int> q;
        q.push(start);
        vis[start] = 1;
        while (q.size())
        {
            int node = q.front();
            q.pop();
            cout << node << " ";
            for (auto adjNode : adj[node])
            {
                if (!vis[adjNode])
                {
                    vis[adjNode] = 1;
                    q.push(adjNode);
                }
            }
        }
    }

    void dfs(int node, vector<int> adj[], int *vis)
    {
        vis[node]++;
        cout << node << " ";
        for (auto adjNode : adj[node])
        {
            if (!vis[adjNode])
                dfs(adjNode, adj, vis);
        }
    }

    bool checkCycleUsingDfs(int prev, int node, vector<bool> &vis, vector<int> adj[])
    {
        vis[node] = true;
        for (auto it : adj[node])
        {
            if (vis[it] && prev != it)
                return true;
            if (!vis[it] && checkCycleUsingDfs(node, it, vis, adj))
                return true;
        }
        return false;
    }

    bool colorAndCheckBipartite(int start, int color[], vector<int> adj[])
    {
        color[start] = 1;
        queue<int> q;
        q.push(start);
        while (!q.empty())
        {
            int node = q.front();
            q.pop();
            for (auto it : adj[node])
            {
                if (color[it] == -1)
                {
                    color[it] = !color[node];
                    q.push(it);
                }
                else if (color[it] == color[node])
                    return false;
            }
        }
        return true;
    }

    void findTopo(int node, vector<int> adj[], vector<bool> &vis, stack<int> &s)
    {
        vis[node] = true;
        for (auto it : adj[node])
        {
            if (!vis[it])
            {
                findTopo(it, adj, vis, s);
            }
        }
        s.push(node);
    }
}

void bfs(int V, vector<int> adj[]) // 1based
{
    int vis[V + 1] = {0};
    for (int i = 1; i <= V; i++)
    {
        if (!vis[i])
        {
            functions::bfs(i, V, adj, vis);
        }
    }
}

void dfs(int V, vector<int> adj[]) // 1based
{
    int vis[V + 1] = {0};
    for (int i = 1; i <= V; i++)
    {
        if (!vis[i])
        {
            functions::dfs(i, adj, vis);
        }
    }
}

bool isCyclePresentBfs(int V, vector<int> adj[]) // 1based
{
    int parent[V + 1];
    memset(parent, -1, sizeof(parent));

    queue<int> q;
    q.push(1);

    parent[1] = -2;

    while (!q.empty())
    {
        int node = q.front();
        q.pop();
        for (auto adjNode : adj[node])
        {
            if (parent[adjNode] == -1)
            {
                q.push(adjNode);
                parent[adjNode] = node;
            }
            else if (parent[node] != adjNode)
            {
                return true;
            }
        }
    }
    return false;
}

bool isCyclePresentDfs(int V, vector<int> adj[]) // 1based
{
    vector<bool> vis(V + 1, false);
    for (int i = 1; i <= V; i++)
    {
        if (!vis[i] && functions::checkCycleUsingDfs(-1, i, vis, adj))
            return true;
    }
    return false;
}
bool isBipartite(int V, vector<int> adj[]) // 1 based
{
    int color[V + 1];
    memset(color, -1, sizeof(color));
    for (int i = 1; i <= V; i++)
    {
        if (color[i] == -1 && !functions::colorAndCheckBipartite(i, color, adj))
            return false;
    }
    return true;
}

void topoSortDfs(int V, vector<int> adj[]) // 0 based
{
    vector<bool> vis(V, false);
    stack<int> s;
    for (int i = 0; i < V; i++)
    {
        if (!vis[i])
        {
            functions::findTopo(i, adj, vis, s);
        }
    }
    cout << "Topological Sort is : ";
    while (!s.empty())
    {
        cout << s.top() << " ";
        s.pop();
    }
    cout << endl;
}

void topoSortKhanAlgo(int V, vector<int> adj[])
{
    int indegree[V] = {0};
    for (int i = 0; i < V; i++)
    {
        for (auto adjNode : adj[i])
        {
            indegree[adjNode]++;
        }
    }
    queue<int> q;
    for (int i = 0; i < V; i++)
    {
        if (indegree[i] == 0)
            q.push(i);
    }
    cout << "Topological Sort is : ";
    while (!q.empty())
    {
        int node = q.front();
        cout << node << " ";
        q.pop();
        for (int adjNode : adj[node])
        {
            indegree[adjNode]--;
            if (indegree[adjNode] == 0)
                q.push(adjNode);
        }
    }
    cout << endl;
}

void dijkstraAlgo(int src, int V, vector<vector<int>> adj[]) // 0based
{

    vector<int> dist(V, 1e8);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq; // distance , current node
    dist[src] = 0;                                                                      // source to source is zero cost
    pq.push({0, src});
    while (!pq.empty())
    {
        int dis = pq.top().first;
        int node = pq.top().second;

        pq.pop();

        for (auto it : adj[node])
        {
            int adjNode = it[0];
            int wt = it[1];

            if (dis + wt < dist[adjNode])
            {
                dist[adjNode] = dis + wt;
                pq.push({dist[adjNode], adjNode});
            }
        }
    }

    for (int i = 0; i < V; i++)
    {
        if (dist[i] == 1e8)
            dist[i] = -1;
        cout << dist[i] << " ";
    }
    cout << endl;
}

void bellmanFordAlgo(int src, int V, vector<vector<int>> adj[]) // 0based
{
    vector<vector<int>> edges;
    vector<int> dist(V, 1e8);
    dist[src] = 0;
    for (int i = 0; i < V; i++)
    {
        for (auto it : adj[i])
        {
            edges.push_back({i, it[0], it[1]});
        }
    }

    for (int i = 0; i < V - 1; i++)
    {
        for (auto it : edges)
        {
            int u = it[0];
            int v = it[1];
            int wt = it[2];
            if (dist[u] != 1e8 && dist[u] + wt < dist[v])
            {
                dist[v] = dist[u] + wt;
            }
        }
    }

    for (int i = 0; i < V; i++)
    {
        if (dist[i] == 1e8)
            dist[i] = -1;
        cout << dist[i] << " ";
    }
    cout << endl;
}

void floydWarshall(int V, vector<vector<int>> adj[]) // 0based and assuming there is no -ve edge weight
{

    vector<vector<int>> matrix(V, vector<int>(V, 1e8));

    for (int i = 0; i < V; i++)
    {
        for (auto it : adj[i])
        {
            matrix[i][it[0]] = it[1];
        }
        matrix[i][i] = 0;
    }

    for (int k = 0; k < V; k++)
    {
        for (int i = 0; i < V; i++)
        {
            for (int j = 0; j < V; j++)
            {
                matrix[i][j] = min(matrix[i][j], matrix[i][k] + matrix[k][j]);
            }
        }
    }

    for (int i = 0; i < V; i++)
    {
        for (int j = 0; j < V; j++)
        {
            if (matrix[i][j] == 1e8)
                matrix[i][j] = -1;
            cout << matrix[i][j] << " ";
        }
        cout << endl;
    }
}

int main()
{
    /*int E, V;
    cout << "Enter Edges and Vertices repectively : ";
    cin >> E >> V;

    vector<int> adj[V + 1];

    for (int i = 0; i < E; i++)
    {
        int u, v;
        cout << "Enter two vertices : ";
        cin >> u >> v;
        adj[u].push_back(v);
        adj[v].push_back(u);
    }
    bfs(V, adj);
    cout << endl;
    dfs(V, adj);
    cout << endl;
    cout << isCyclePresentBfs(V, adj) << " " << isCyclePresentDfs(V, adj) << " " << isBipartite(V, adj);
    */

    /*int V, E;
     cin >> V >> E;
     vector<int> adj[V];
     for (int i = 0; i < E; i++)
     {
         int u, v;
         cin >> u >> v;
         adj[u].push_back(v);
     }
     topoSortDfs(V, adj);
     topoSortKhanAlgo(V, adj);
     */

    int V, E;
    cin >> V >> E;
    vector<vector<int>> adj[V];
    for (int i = 0; i < E; i++)
    {
        int u, v, wt;
        cin >> u >> v >> wt;
        adj[u].push_back({v, wt});
        //  adj[v].push_back({u, wt});
    }
    dijkstraAlgo(0, V, adj);
    bellmanFordAlgo(0, V, adj);
    cout << endl;
    floydWarshall(V, adj);
    return 0;
}