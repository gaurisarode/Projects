<%@ include file="header.jsp" %>
<%@ include file="sidebar.jsp" %>
<sql:query dataSource="${ds}" var="result">
SELECT * from users;
</sql:query> 
<section id="main-content">
<section class="wrapper">
<font color="red" ><strong><c:out value="${msg }"></c:out></strong></font>
 <div class="row">
                  <div class="col-lg-12">
                      <section class="panel">
                          <header class="panel-heading">
                              Manage Category                          </header>
                          
                          <table class="table table-striped table-advance table-hover">
                           <tbody>
                              <tr>
                                 <th width="15%"><i class="icon_profile"></i> Name </th>
                                 <th width="15%"><i class="icon_profile"></i> Email </th>
                                 <th width="13%"><i class="icon_profile"></i> City </th>
                                 <th width="15%"><i class="icon_profile"></i> Password </th>
                                 <th width="15%"><i class="icon_profile"></i> Contact </th>
                                 <th width="14%"><i class="icon_profile"></i> Priviledge </th>
                                 <th width="13"><i class="icon_cogs"></i> Action</th>
                              </tr>
                              <c:forEach var="row" items="${result.rows}">
                              <tr>
                                 <td><c:out value="${row.name}"/></td>
                                  <td><c:out value="${row.email}"/></td>
                                   <td><c:out value="${row.city}"/></td>
                                    <td><c:out value="${row.password}"/></td>
                                     <td><c:out value="${row.contact}"/></td>
                                      <td align="center"><c:out value="${row.priviledge}"/></td> 
                                 <td>
                                  <div class="btn-group">
                                      <a class="btn btn-primary" href="<%=request.getContextPath() %>/Admin?action=user-edit&id=<c:out value="${row.id}"/>"><i class="icon_plus_alt2"></i></a>
                                      <a class="btn btn-danger" href="<%=request.getContextPath() %>/Admin?action=user-delete&id=<c:out value="${row.id}"/>"><i class="icon_close_alt2"></i></a>                                  </div>                                </td>
                              </tr>  
                              </c:forEach>                            
                           </tbody>
                        </table>
                      </section>
                  </div>
              </div>
</section>
</section>
<%@ include file="footer.jsp" %>